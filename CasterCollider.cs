using UnityEngine;

public class CasterCollider : MonoBehaviour
{
    public Rigidbody parentRigidbody;
    public float radius; //The radius of the wheel, measured in local space.
    public float groundDetectorDistance;
    public LayerMask groundLayers;
    public float mass; //The mass of the wheel. Must be larger than zero.

    Transform _dummyWheel;
    WheelFrictionCurve _forwardFriction; //Properties of tire friction in the direction the wheel is pointing in.
    WheelFrictionCurve _sidewaysFriction; //Properties of tire friction in the sideways direction.
    float _forwardSlip;
    float _sidewaysSlip;
    Vector3 _totalForce;
    Vector3 _prevPosition;
    bool _isGrounded; //Indicates whether the wheel currently collides with something.
    float _wheelMotorTorque; //Motor torque on the wheel axle. Positive or negative depending on direction.
    float _wheelBrakeTorque; //Brake torque. Must be positive.
    float _wheelSteerAngle; //Steering angle in degrees, always around the local y-axis.
    float _wheelAngularVelocity; //Current wheel axle rotation speed, in rotations per minute.
    float _wheelRotationAngle;
    RaycastHit _hit;

    // Use this for initialization
    void Awake()
    {
        _dummyWheel = new GameObject("DummyWheel").transform;
        _dummyWheel.transform.parent = this.transform.parent;

        _dummyWheel.localPosition = transform.localPosition;
        _prevPosition = _dummyWheel.localPosition;

        _forwardFriction = new WheelFrictionCurve();
        _sidewaysFriction = new WheelFrictionCurve();
    }

    // Called once per physics update
    void FixedUpdate()
    {
        //Raycast down along the suspension to find out how far the ground is to the wheel
        if (Physics.Raycast(new Ray(_dummyWheel.position, -_dummyWheel.up), out _hit, groundDetectorDistance, groundLayers)) //The wheel is in contact with the ground
        {
            if (!_isGrounded) //If not previously grounded, set the prevPosition value to the wheel's current position.
            {
                _prevPosition = _dummyWheel.position;
            }
            _isGrounded = true;
        }
        else //The wheel is in the air
        {
            _isGrounded = false;
        }

        //Set steering angle of the wheel dummy
        _dummyWheel.localEulerAngles = new Vector3(0, _wheelSteerAngle, 0);
        //Calculate the wheel's rotation given it's angular velocity
        _wheelRotationAngle += _wheelAngularVelocity * Time.deltaTime;
        //Set the rotation and steer angle of the wheel model
        this.transform.localEulerAngles = new Vector3(_wheelRotationAngle, _wheelSteerAngle, 0);

        //Apply rolling force to tires if they are grounded and don't have motor torque applied to them.
        if (_isGrounded && _wheelMotorTorque == 0)
        {
            //Apply angular force to wheel from slip
            _wheelAngularVelocity -= Mathf.Sign(_forwardSlip) * _forwardFriction.Evaluate(_forwardSlip) / (Mathf.PI  * 2.0f * radius) / mass * Time.deltaTime;
        }
        //Apply motor torque
        _wheelAngularVelocity += _wheelMotorTorque / radius / mass * Time.deltaTime;
        //Apply brake torque
        _wheelAngularVelocity -= Mathf.Sign(_wheelAngularVelocity) * Mathf.Min(Mathf.Abs(_wheelAngularVelocity), _wheelBrakeTorque * radius / mass * Time.deltaTime);

        if (_isGrounded && parentRigidbody != null)
        {
            //Calculate the wheel's linear velocity
            Vector3 velocity = (transform.position - _prevPosition) / Time.fixedDeltaTime;
            _prevPosition = transform.position;
            //Store the forward and sideways direction to improve performance
            Vector3 forward = _dummyWheel.forward;
            Vector3 sideways = -_dummyWheel.right;
            //Calculate the forward and sideways velocity components relative to the wheel
            Vector3 forwardVelocity = Vector3.Dot(velocity, forward) * forward;
            Vector3 sidewaysVelocity = Vector3.Dot(velocity, sideways) * sideways;
            //Calculate the slip velocities. Note that these values are different from the standard slip calculation.
            _forwardSlip = -Mathf.Sign(Vector3.Dot(forward, forwardVelocity)) * forwardVelocity.magnitude + (_wheelAngularVelocity * Mathf.PI / 180.0f * radius);
            _sidewaysSlip = -Mathf.Sign(Vector3.Dot(sideways, sidewaysVelocity)) * sidewaysVelocity.magnitude;
            //Forward slip force
            _totalForce = _dummyWheel.forward * Mathf.Sign(_forwardSlip) * _forwardFriction.Evaluate(_forwardSlip);
            //Lateral slip force
            _totalForce -= _dummyWheel.right * Mathf.Sign(_sidewaysSlip) * _forwardFriction.Evaluate(_sidewaysSlip);

            parentRigidbody.AddForceAtPosition(_totalForce, transform.position);
        }
    }

    public void OnDrawGizmosSelected()
    {
        Transform t = _dummyWheel != null ? _dummyWheel.transform : transform;
        Gizmos.color = Physics.Raycast(new Ray(t.position, -t.up), out _hit, groundDetectorDistance, groundLayers) ? Color.blue : Color.green;
        Gizmos.DrawLine(t.position, t.position - t.up * groundDetectorDistance);
        Gizmos.DrawWireSphere(t.position - t.up * groundDetectorDistance, .1f);

        //Draw the wheel
        Vector3 point1;
        Vector3 point0 = transform.TransformPoint(radius * new Vector3(0, Mathf.Sin(0), Mathf.Cos(0)));
        for (int i = 1; i <= 20; ++i)
        {
            point1 = transform.TransformPoint(radius * new Vector3(0, Mathf.Sin(i / 20.0f * Mathf.PI * 2.0f), Mathf.Cos(i / 20.0f * Mathf.PI * 2.0f)));
            Gizmos.DrawLine(point0, point1);
            point0 = point1;
        }
    }

    //Standard accessor and mutator properties
    public float MotorTorque
    {
        set
        {
            _wheelMotorTorque = value;
        }
        get
        {
            return _wheelMotorTorque;
        }
    }

    public float BrakeTorque
    {
        set
        {
            _wheelBrakeTorque = value;
        }
        get
        {
            return _wheelBrakeTorque;
        }
    }

    public float SteerAngle
    {
        set
        {
            _wheelSteerAngle = value;
        }
        get
        {
            return _wheelSteerAngle;
        }
    }
}

public class WheelFrictionCurve
{
    struct WheelFrictionCurvePoint
    {
        public float TValue;
        public Vector2 SlipForcePoint;
    }
    float _extremumSlip; //Extremum point slip (default 3).
    float _extremumValue; //Force at the extremum slip (default 6000).
    float _asymptoteSlip; //Asymptote point slip (default 4).
    float _asymptoteValue; //Force at the asymptote slip (default 5500).
    float _stiffness; //Multiplier for the extremumValue and asymptoteValue values (default 1).

    int _arraySize;
    WheelFrictionCurvePoint[] _extremePoints;
    WheelFrictionCurvePoint[] _asymptotePoints;

    public WheelFrictionCurve()
    {
        _extremumSlip = 3;
        _extremumValue = 6000;
        _asymptoteSlip = 4;
        _asymptoteValue = 5500;
        _stiffness = 1;

        //Generate the arrays
        _arraySize = 50;
        _extremePoints = new WheelFrictionCurvePoint[_arraySize];
        _asymptotePoints = new WheelFrictionCurvePoint[_arraySize];

        for (int i = 0; i < _arraySize; ++i)
        {
            _extremePoints[i].TValue = (float)i / (float)_arraySize;
            _extremePoints[i].SlipForcePoint = Hermite(
                    (float)i / (float)_arraySize,
                    Vector2.zero,
                    new Vector2(_extremumSlip, _extremumValue),
                    Vector2.zero,
                    new Vector2(_extremumSlip * 0.5f + 1, 0)
                );
            _asymptotePoints[i].TValue = (float)i / (float)_arraySize;
            _asymptotePoints[i].SlipForcePoint = Hermite(
                (float)i / (float)_arraySize,
                    new Vector2(_extremumSlip, _extremumValue),
                    new Vector2(_asymptoteSlip, _asymptoteValue),
                    new Vector2((_asymptoteSlip - _extremumSlip) * 0.5f + 1, 0),
                    new Vector2((_asymptoteSlip - _extremumSlip) * 0.5f + 1, 0)
                );
        }
    }

    public float Evaluate(float slip)
    {
        //The slip value must be positive.
        slip = Mathf.Abs(slip);

        if (slip < _extremumSlip)
        {
            return Evaluate(slip, _extremePoints) * _stiffness;
        }
        else if (slip < _asymptoteSlip)
        {
            return Evaluate(slip, _asymptotePoints) * _stiffness;
        }
        else
        {
            return _asymptoteValue * _stiffness;
        }
    }

    float Evaluate(float slip, WheelFrictionCurvePoint[] curvePoints)
    {
        int top = _arraySize - 1;
        int bottom = 0;
        int index = (int)((top + bottom) * 0.5f);
        WheelFrictionCurvePoint result = curvePoints[index];

        //Binary search the curve to find the corresponding t value for the given slip
        while ((top != bottom && top - bottom > 1))
        {
            if (result.SlipForcePoint.x <= slip)
            {
                bottom = index;
            }
            else if (result.SlipForcePoint.x >= slip)
            {
                top = index;
            }

            index = (int)((top + bottom) * 0.5f);
            result = curvePoints[index];
        }

        float slip1 = curvePoints[bottom].SlipForcePoint.x;
        float slip2 = curvePoints[top].SlipForcePoint.x;
        float force1 = curvePoints[bottom].SlipForcePoint.y;
        float force2 = curvePoints[top].SlipForcePoint.y;
        float slipFraction = (slip - slip1) / (slip2 - slip1);

        return force1 * (1- slipFraction) + force2 * (slipFraction);
    }

    Vector2 Hermite(float t, Vector2 p0, Vector2 p1, Vector2 m0, Vector2 m1)
    {
        float t2 = t * t;
        float t3 = t2 * t;

        return
            (2 * t3 - 3 * t2 + 1) * p0 +
            (t3 - 2 * t2 + t) * m0 +
            (-2 * t3 + 3 * t2) * p1 +
            (t3 - t2) * m1;
    }
}