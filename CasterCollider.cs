using UnityEngine;
 
//[RequireComponent(typeof(SphereCollider))]
public class CasterCollider : MonoBehaviour
{
	public float radius;
	public float groundDetectorRadius;
	public LayerMask groundLayers;
	public float forwardFriction;
	public float sidewaysFriction;
	public float brakeForce;
	public float motorTorque;
	public float steerAngle;
	public Rigidbody parentRigidbody;

	RaycastHit _hit;
	Collider[] _hits;
	float _forwardSlip, _sidewaysSlip;
	Vector3 _lastPosition, _velocity, _forwardVelocity, _sidewaysVelocity, _forward, _right, _force;

	void Awake()
	{
		_lastPosition = transform.position;
		_hits = new Collider[2];
	}

	void FixedUpdate()
	{
		if (parentRigidbody != null && Physics.OverlapSphereNonAlloc(transform.position + transform.up * -radius, groundDetectorRadius, _hits, groundLayers) > 0)
		{
			// store the forward and sideways direction to improve performance
			//_forward = Quaternion.AngleAxis(steerAngle, Vector3.up) * transform.forward;
			_forward = Quaternion.AngleAxis(steerAngle, transform.up) * transform.forward;
			_right = Quaternion.AngleAxis(steerAngle, transform.up) * transform.right;

			// calculate the wheel's linear velocity
			_velocity = (transform.position - _lastPosition) / Time.fixedDeltaTime;
			_lastPosition = transform.position;
	
			// calculate the forward and sideways velocity components relative to the wheel
			_forwardVelocity = Vector3.Dot(_velocity, _forward) * _forward;
			_sidewaysVelocity = Vector3.Dot(_velocity, _right) * _right;
	
			// calculate the slip velocities; note that these values are different from the standard slip calculation
			_forwardSlip = Mathf.Sign(Vector3.Dot(_forward, _forwardVelocity)) * _forwardVelocity.magnitude; // - motorTorque; // + (wheelAngularVelocity * Mathf.PI / 180f * radius);
			_sidewaysSlip = Mathf.Sign(Vector3.Dot(_right, _sidewaysVelocity)) * _sidewaysVelocity.magnitude; // - motorTorque * (sidewaysFriction / forwardFriction);

			_force = new Vector3();
			// forward friction
			_force += _forward * -_forwardSlip * forwardFriction;
			// sideways friction
			_force += _right * -_sidewaysSlip * sidewaysFriction;
			// forward force
			_force += _forward * motorTorque;
			// brake force
			_force -= _forward * brakeForce;
			parentRigidbody.AddForceAtPosition(_force, transform.position, ForceMode.Impulse);
		}
	}

	void OnDrawGizmosSelected()
	{
		Gizmos.color = Physics.OverlapSphereNonAlloc(transform.position + transform.up * -radius, groundDetectorRadius, _hits, groundLayers) > 0 ? Color.blue : Color.green;
		Gizmos.DrawLine(transform.position, transform.position + transform.up * -radius);
		Gizmos.DrawWireSphere(transform.position + transform.up * -radius, groundDetectorRadius);

		Gizmos.color = Color.blue;
		Gizmos.DrawLine(transform.position, transform.position + Quaternion.AngleAxis(steerAngle, transform.up) * transform.forward * 2f);
		Gizmos.DrawCube(transform.position + Quaternion.AngleAxis(steerAngle, transform.up) * transform.forward * 2f, new Vector3(.1f, .1f, .1f));
	}
}

