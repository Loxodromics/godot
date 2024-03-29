/**************************************************************************/
/*  mobile_vr_interface.cpp                                               */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "mobile_vr_interface.h"

#include "core/input/input.h"
#include "core/math/vector3.h"
#include "core/os/os.h"
#include "core/string/print_string.h"
#include "modules/mobile_vr/FusionAhrs.h"
#include "modules/mobile_vr/FusionMath.h"
#include "servers/display_server.h"
#include "servers/rendering/rendering_server_globals.h"

StringName MobileVRInterface::get_name() const {
	return "Native mobile";
};

uint32_t MobileVRInterface::get_capabilities() const {
	return XRInterface::XR_STEREO;
};

Vector3 MobileVRInterface::scale_magneto(const Vector3 &p_magnetometer) {
	// Our magnetometer doesn't give us nice clean data.
	// Well it may on macOS because we're getting a calibrated value in the current implementation but Android we're getting raw data.
	// This is a fairly simple adjustment we can do to correct for the magnetometer data being elliptical

	Vector3 mag_raw = p_magnetometer;
	Vector3 mag_scaled = p_magnetometer;

	// update our variables every x frames
	if (mag_count > 20) {
		mag_current_min = mag_next_min;
		mag_current_max = mag_next_max;
		mag_count = 0;
	} else {
		mag_count++;
	};

	// adjust our min and max
	if (mag_raw.x > mag_next_max.x) {
		mag_next_max.x = mag_raw.x;
	}
	if (mag_raw.y > mag_next_max.y) {
		mag_next_max.y = mag_raw.y;
	}
	if (mag_raw.z > mag_next_max.z) {
		mag_next_max.z = mag_raw.z;
	}

	if (mag_raw.x < mag_next_min.x) {
		mag_next_min.x = mag_raw.x;
	}
	if (mag_raw.y < mag_next_min.y) {
		mag_next_min.y = mag_raw.y;
	}
	if (mag_raw.z < mag_next_min.z) {
		mag_next_min.z = mag_raw.z;
	}

	// scale our x, y and z
	if (!(mag_current_max.x - mag_current_min.x)) {
		mag_raw.x -= (mag_current_min.x + mag_current_max.x) / 2.0;
		mag_scaled.x = (mag_raw.x - mag_current_min.x) / ((mag_current_max.x - mag_current_min.x) * 2.0 - 1.0);
	};

	if (!(mag_current_max.y - mag_current_min.y)) {
		mag_raw.y -= (mag_current_min.y + mag_current_max.y) / 2.0;
		mag_scaled.y = (mag_raw.y - mag_current_min.y) / ((mag_current_max.y - mag_current_min.y) * 2.0 - 1.0);
	};

	if (!(mag_current_max.z - mag_current_min.z)) {
		mag_raw.z -= (mag_current_min.z + mag_current_max.z) / 2.0;
		mag_scaled.z = (mag_raw.z - mag_current_min.z) / ((mag_current_max.z - mag_current_min.z) * 2.0 - 1.0);
	};

	return mag_scaled;
};

Basis MobileVRInterface::combine_acc_mag(const Vector3 &p_grav, const Vector3 &p_magneto) {
	// yup, stock standard cross product solution...
	Vector3 up = -p_grav.normalized();

	Vector3 magneto_east = up.cross(p_magneto.normalized()); // or is this west?, but should be horizon aligned now
	magneto_east.normalize();

	Vector3 magneto = up.cross(magneto_east); // and now we have a horizon aligned north
	magneto.normalize();

	// We use our gravity and magnetometer vectors to construct our matrix
	Basis acc_mag_m3;
	acc_mag_m3.rows[0] = -magneto_east;
	acc_mag_m3.rows[1] = up;
	acc_mag_m3.rows[2] = magneto;

	return acc_mag_m3 * -1.0f;
};

void MobileVRInterface::set_position_from_sensors() {
	_THREAD_SAFE_METHOD_

	// this is a helper function that attempts to adjust our transform using our 9dof sensors
	// 9dof is a misleading marketing term coming from 3 accelerometer axis + 3 gyro axis + 3 magnetometer axis = 9 axis
	// but in reality this only offers 3 dof (yaw, pitch, roll) orientation

	uint64_t ticks = OS::get_singleton()->get_ticks_usec();
	uint64_t ticks_elapsed = ticks - last_ticks;
	float delta_time = (double)ticks_elapsed / 1000000.0;

	// few things we need
	Input *input = Input::get_singleton();
	Vector3 down(0.0, -1.0, 0.0); // Down is Y negative
	Vector3 north(0.0, 0.0, 1.0); // North is Z positive


	// make copies of our inputs
	Vector3 acc = input->get_accelerometer();
	acc *= -1.0f;
	Vector3 gyro = input->get_gyroscope();
	Vector3 grav = input->get_gravity();
	Vector3 magneto = scale_magneto(input->get_magnetometer()); // this may be overkill on iOS because we're already getting a calibrated magnetometer reading


	// Print statements for debugging
	print_line("Accelerometer: (" + String::num_real(acc.x) + ", " + String::num_real(acc.y) + ", " + String::num_real(acc.z) + ")");
	print_line("Gyroscope: (" + String::num_real(gyro.x) + ", " + String::num_real(gyro.y) + ", " + String::num_real(gyro.z) + ")");
	print_line("Gravity: (" + String::num_real(grav.x) + ", " + String::num_real(grav.y) + ", " + String::num_real(grav.z) + ")");
	print_line("Mageneto: (" + String::num_real(magneto.x) + ", " + String::num_real(magneto.y) + ", " + String::num_real(magneto.z) + ")");

	static const float PI = 3.141592653589793238f;
	// FusionVector gyroscope = {0.0f, 0.0f, 0.0f};
	FusionVector gyroscope = {gyro.x * PI / 180.0f, gyro.y * PI / 180.0f, gyro.z * PI / 180.0f}; // replace this with actual gyroscope data in degrees/s
	// FusionVector gyroscope = {gyro.x, gyro.z, gyro.y}; // replace this with actual gyroscope data in degrees/s
	FusionVector accelerometer = {acc.x, acc.z, acc.y}; // replace this with actual accelerometer data in g
	FusionVector magnetometer = {magneto.x, magneto.y, magneto.z};
	// Update Madgwick filter
	// FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, delta_time);


	// // Apply calibration
	// gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
	// accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
	// magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

	// // Update gyroscope offset correction algorithm
	// gyroscope = FusionOffsetUpdate(&offset, gyroscope);


	FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, delta_time);
	FusionAhrsInternalStates state = FusionAhrsGetInternalStates(&ahrs);

	print_line("accelerationError: " + String::num_real(state.accelerationError) + " accelerometerIgnored: " + String::num_real( state.accelerometerIgnored) +
		" accelerationRecoveryTrigger: " + String::num_real(state.accelerationRecoveryTrigger) + " magneticError: " + String::num_real(state.magneticError) +
		" magnetometerIgnored: " + String::num_real(state.magnetometerIgnored) + " magneticRecoveryTrigger: " + String::num_real(state.magneticRecoveryTrigger));

	FusionQuaternion quad = FusionAhrsGetQuaternion(&ahrs);
	Quaternion q {quad.element.x, quad.element.y, quad.element.z, quad.element.w};

	// Convert the filter's quaternion output to a Basis
	// Quaternion q = madgwickFilter.get_orientation();
	// Quaternion rotationQuaternion = Quaternion(acc.normalized(), -gyro.y);
	// orientation = orientation.lerp(Basis(q), 0.5f);
	orientation = Basis(q);

	// acc = Vector3(0.0f, 1.0f, 0.0f);
	acc = acc.normalized();
	// acc = Vector3(acc.z, acc.y, acc.x);
	// Choose an arbitrary forward vector, ensuring it's not parallel to the up vector
	Vector3 forwardVector =  Vector3(0.0, 0.0, 1.0);
	// Calculate the right vector as the cross product of upVector and forwardVector
	Vector3 rightVector = acc.cross(forwardVector).normalized();

	// Recalculate the forward vector to ensure orthogonality
	forwardVector = rightVector.cross(acc).normalized();

	// Create a new Basis from the right, up, and forward vectors
	Basis orientationBasis(rightVector, acc, forwardVector);

	// First, extract each column vector from the orientationBasis
	Vector3 column0 = orientationBasis.get_column(0); // Right vector
	Vector3 column1 = orientationBasis.get_column(1); // Up vector
	Vector3 column2 = orientationBasis.get_column(2); // Forward vector

	// Now, print each column vector
	print_line("acc");
	print_line("Right vector: " + column0);
	print_line("Up vector: " + column1);
	print_line("Forward vector: " + column2);

	column0 = orientation.get_column(0); // Right vector
	column1 = orientation.get_column(1); // Up vector
	column2 = orientation.get_column(2); // Forward vector

	// Now, print each column vector
	print_line("Fusion");
	print_line("Right vector: " + column0);
	print_line("Up vector: " + column1);
	print_line("Forward vector: " + column2);

	// and copy to our head transform
	head_transform.basis = orientation.orthonormalized();
	// head_transform.basis = orientationBasis; //.orthonormalized();

	last_ticks = ticks;
};

void MobileVRInterface::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_eye_height", "eye_height"), &MobileVRInterface::set_eye_height);
	ClassDB::bind_method(D_METHOD("get_eye_height"), &MobileVRInterface::get_eye_height);

	ClassDB::bind_method(D_METHOD("set_iod", "iod"), &MobileVRInterface::set_iod);
	ClassDB::bind_method(D_METHOD("get_iod"), &MobileVRInterface::get_iod);

	ClassDB::bind_method(D_METHOD("set_display_width", "display_width"), &MobileVRInterface::set_display_width);
	ClassDB::bind_method(D_METHOD("get_display_width"), &MobileVRInterface::get_display_width);

	ClassDB::bind_method(D_METHOD("set_display_to_lens", "display_to_lens"), &MobileVRInterface::set_display_to_lens);
	ClassDB::bind_method(D_METHOD("get_display_to_lens"), &MobileVRInterface::get_display_to_lens);

	ClassDB::bind_method(D_METHOD("set_oversample", "oversample"), &MobileVRInterface::set_oversample);
	ClassDB::bind_method(D_METHOD("get_oversample"), &MobileVRInterface::get_oversample);

	ClassDB::bind_method(D_METHOD("set_k1", "k"), &MobileVRInterface::set_k1);
	ClassDB::bind_method(D_METHOD("get_k1"), &MobileVRInterface::get_k1);

	ClassDB::bind_method(D_METHOD("set_k2", "k"), &MobileVRInterface::set_k2);
	ClassDB::bind_method(D_METHOD("get_k2"), &MobileVRInterface::get_k2);

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "eye_height", PROPERTY_HINT_RANGE, "0.0,3.0,0.1"), "set_eye_height", "get_eye_height");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "iod", PROPERTY_HINT_RANGE, "4.0,10.0,0.1"), "set_iod", "get_iod");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "display_width", PROPERTY_HINT_RANGE, "5.0,25.0,0.1"), "set_display_width", "get_display_width");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "display_to_lens", PROPERTY_HINT_RANGE, "5.0,25.0,0.1"), "set_display_to_lens", "get_display_to_lens");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "oversample", PROPERTY_HINT_RANGE, "1.0,2.0,0.1"), "set_oversample", "get_oversample");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "k1", PROPERTY_HINT_RANGE, "0.1,10.0,0.0001"), "set_k1", "get_k1");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "k2", PROPERTY_HINT_RANGE, "0.1,10.0,0.0001"), "set_k2", "get_k2");
}

void MobileVRInterface::set_eye_height(const double p_eye_height) {
	eye_height = p_eye_height;
}

double MobileVRInterface::get_eye_height() const {
	return eye_height;
}

void MobileVRInterface::set_iod(const double p_iod) {
	intraocular_dist = p_iod;
};

double MobileVRInterface::get_iod() const {
	return intraocular_dist;
};

void MobileVRInterface::set_display_width(const double p_display_width) {
	display_width = p_display_width;
};

double MobileVRInterface::get_display_width() const {
	return display_width;
};

void MobileVRInterface::set_display_to_lens(const double p_display_to_lens) {
	display_to_lens = p_display_to_lens;
};

double MobileVRInterface::get_display_to_lens() const {
	return display_to_lens;
};

void MobileVRInterface::set_oversample(const double p_oversample) {
	oversample = p_oversample;
};

double MobileVRInterface::get_oversample() const {
	return oversample;
};

void MobileVRInterface::set_k1(const double p_k1) {
	k1 = p_k1;
};

double MobileVRInterface::get_k1() const {
	return k1;
};

void MobileVRInterface::set_k2(const double p_k2) {
	k2 = p_k2;
};

double MobileVRInterface::get_k2() const {
	return k2;
};

uint32_t MobileVRInterface::get_view_count() {
	// needs stereo...
	return 2;
};

XRInterface::TrackingStatus MobileVRInterface::get_tracking_status() const {
	return tracking_state;
}

bool MobileVRInterface::is_initialized() const {
	return (initialized);
};

bool MobileVRInterface::initialize() {
	XRServer *xr_server = XRServer::get_singleton();
	ERR_FAIL_NULL_V(xr_server, false);

	if (!initialized) {
		// reset our sensor data
		mag_count = 0;
		has_gyro = false;
		sensor_first = true;
		mag_next_min = Vector3(10000, 10000, 10000);
		mag_next_max = Vector3(-10000, -10000, -10000);
		mag_current_min = Vector3(0, 0, 0);
		mag_current_max = Vector3(0, 0, 0);
		head_transform.basis = Basis();
		head_transform.origin = Vector3(0.0, eye_height, 0.0);

		// we must create a tracker for our head
		head.instantiate();
		head->set_tracker_type(XRServer::TRACKER_HEAD);
		head->set_tracker_name("head");
		head->set_tracker_desc("Players head");
		xr_server->add_tracker(head);

		// make this our primary interface
		xr_server->set_primary_interface(this);

		last_ticks = OS::get_singleton()->get_ticks_usec();

		// Define calibration (replace with actual calibration data if available)
		const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
		const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
		const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
		const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
		const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
		const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
		const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
		const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

		// Initialise algorithms
		FusionOffset offset;

		FusionOffsetInitialise(&offset, 30);

		// Set AHRS algorithm settings
		const FusionAhrsSettings settings = {
				.convention = FusionConventionNwu,
				.gain = 0.5f,
				.gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
				.accelerationRejection = 0.0f,
				.magneticRejection = 0.0f,
				.recoveryTriggerPeriod = 1 * 30, /* 5 seconds */
		};
		FusionAhrsSetSettings(&ahrs, &settings);
		// FusionAhrsInitialise(&ahrs);

		initialized = true;
	};

	return true;
};

void MobileVRInterface::uninitialize() {
	if (initialized) {
		// do any cleanup here...
		XRServer *xr_server = XRServer::get_singleton();
		if (xr_server != nullptr) {
			if (head.is_valid()) {
				xr_server->remove_tracker(head);

				head.unref();
			}

			if (xr_server->get_primary_interface() == this) {
				// no longer our primary interface
				xr_server->set_primary_interface(nullptr);
			}
		}

		initialized = false;
	};
};

Dictionary MobileVRInterface::get_system_info() {
	Dictionary dict;

	dict[SNAME("XRRuntimeName")] = String("Godot mobile VR interface");
	dict[SNAME("XRRuntimeVersion")] = String("");

	return dict;
}

bool MobileVRInterface::supports_play_area_mode(XRInterface::PlayAreaMode p_mode) {
	// This interface has no positional tracking so fix this to 3DOF
	return p_mode == XR_PLAY_AREA_3DOF;
}

XRInterface::PlayAreaMode MobileVRInterface::get_play_area_mode() const {
	return XR_PLAY_AREA_3DOF;
}

bool MobileVRInterface::set_play_area_mode(XRInterface::PlayAreaMode p_mode) {
	return p_mode == XR_PLAY_AREA_3DOF;
}

Size2 MobileVRInterface::get_render_target_size() {
	_THREAD_SAFE_METHOD_

	// we use half our window size
	Size2 target_size = DisplayServer::get_singleton()->window_get_size();

	target_size.x *= 0.5 * oversample;
	target_size.y *= oversample;

	return target_size;
};

Transform3D MobileVRInterface::get_camera_transform() {
	_THREAD_SAFE_METHOD_

	Transform3D transform_for_eye;

	XRServer *xr_server = XRServer::get_singleton();
	ERR_FAIL_NULL_V(xr_server, transform_for_eye);

	if (initialized) {
		float world_scale = xr_server->get_world_scale();

		// just scale our origin point of our transform
		Transform3D _head_transform = head_transform;
		_head_transform.origin *= world_scale;

		transform_for_eye = (xr_server->get_reference_frame()) * _head_transform;
	}

	return transform_for_eye;
};

Transform3D MobileVRInterface::get_transform_for_view(uint32_t p_view, const Transform3D &p_cam_transform) {
	_THREAD_SAFE_METHOD_

	Transform3D transform_for_eye;

	XRServer *xr_server = XRServer::get_singleton();
	ERR_FAIL_NULL_V(xr_server, transform_for_eye);

	if (initialized) {
		float world_scale = xr_server->get_world_scale();

		// we don't need to check for the existence of our HMD, doesn't affect our values...
		// note * 0.01 to convert cm to m and * 0.5 as we're moving half in each direction...
		if (p_view == 0) {
			transform_for_eye.origin.x = -(intraocular_dist * 0.01 * 0.5 * world_scale);
		} else if (p_view == 1) {
			transform_for_eye.origin.x = intraocular_dist * 0.01 * 0.5 * world_scale;
		} else {
			// should not have any other values..
		};

		// just scale our origin point of our transform
		Transform3D _head_transform = head_transform;
		_head_transform.origin *= world_scale;

		transform_for_eye = p_cam_transform * (xr_server->get_reference_frame()) * _head_transform * transform_for_eye;
	} else {
		// huh? well just return what we got....
		transform_for_eye = p_cam_transform;
	};

	return transform_for_eye;
};

Projection MobileVRInterface::get_projection_for_view(uint32_t p_view, double p_aspect, double p_z_near, double p_z_far) {
	_THREAD_SAFE_METHOD_

	Projection eye;

	aspect = p_aspect;
	eye.set_for_hmd(p_view + 1, p_aspect, intraocular_dist, display_width, display_to_lens, oversample, p_z_near, p_z_far);

	return eye;
};

Vector<BlitToScreen> MobileVRInterface::post_draw_viewport(RID p_render_target, const Rect2 &p_screen_rect) {
	_THREAD_SAFE_METHOD_

	Vector<BlitToScreen> blit_to_screen;

	// We must have a valid render target
	ERR_FAIL_COND_V(!p_render_target.is_valid(), blit_to_screen);

	// Because we are rendering to our device we must use our main viewport!
	ERR_FAIL_COND_V(p_screen_rect == Rect2(), blit_to_screen);

	// and add our blits
	BlitToScreen blit;
	blit.render_target = p_render_target;
	blit.multi_view.use_layer = true;
	blit.lens_distortion.apply = true;
	blit.lens_distortion.k1 = k1;
	blit.lens_distortion.k2 = k2;
	blit.lens_distortion.upscale = oversample;
	blit.lens_distortion.aspect_ratio = aspect;

	// left eye
	blit.dst_rect = p_screen_rect;
	blit.dst_rect.size.width *= 0.5;
	blit.multi_view.layer = 0;
	blit.lens_distortion.eye_center.x = ((-intraocular_dist / 2.0) + (display_width / 4.0)) / (display_width / 2.0);
	blit_to_screen.push_back(blit);

	// right eye
	blit.dst_rect = p_screen_rect;
	blit.dst_rect.size.width *= 0.5;
	blit.dst_rect.position.x = blit.dst_rect.size.width;
	blit.multi_view.layer = 1;
	blit.lens_distortion.eye_center.x = ((intraocular_dist / 2.0) - (display_width / 4.0)) / (display_width / 2.0);
	blit_to_screen.push_back(blit);

	return blit_to_screen;
}

void MobileVRInterface::process() {
	_THREAD_SAFE_METHOD_

	if (initialized) {
		// update our head transform orientation
		set_position_from_sensors();

		// update our head transform position (should be constant)
		head_transform.origin = Vector3(0.0, eye_height, 0.0);

		if (head.is_valid()) {
			// Set our head position, note in real space, reference frame and world scale is applied later
			head->set_pose("default", head_transform, Vector3(), Vector3(), tracking_confidence);
		}
	};
};

MobileVRInterface::MobileVRInterface() {}

MobileVRInterface::~MobileVRInterface() {
	// and make sure we cleanup if we haven't already
	if (is_initialized()) {
		uninitialize();
	};
};
