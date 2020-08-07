#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Tool functions to convert transforms from carla to icv coordinate system
"""

import math
import numpy

import transformations
from geometry_msgs.msg import Vector3, Quaternion, Transform, Pose, Point, Twist, Accel


def carla_location_to_numpy_vector(carla_location):
    """
    Convert a carla location to a icv vector3

    Considers the conversion from left-handed system (unreal) to right-handed
    system (icv)

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a numpy.array with 3 elements
    :rtype: numpy.array
    """
    return numpy.array([
        carla_location.x,
        -carla_location.y,
        carla_location.z
    ])


def carla_location_to_icv_vector3(carla_location):
    """
    Convert a carla location to a icv vector3

    Considers the conversion from left-handed system (unreal) to right-handed
    system (icv)

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a icv vector3
    :rtype: geometry_msgs.msg.Vector3
    """
    icv_translation = Vector3()
    icv_translation.x = carla_location.x
    icv_translation.y = -carla_location.y
    icv_translation.z = carla_location.z

    return icv_translation


def carla_location_to_icv_point(carla_location):
    """
    Convert a carla location to a icv point

    Considers the conversion from left-handed system (unreal) to right-handed
    system (icv)

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a icv point
    :rtype: geometry_msgs.msg.Point
    """
    icv_point = Point()
    icv_point.x = carla_location.x
    icv_point.y = -carla_location.y
    icv_point.z = carla_location.z

    return icv_point


def numpy_quaternion_to_icv_quaternion(numpy_quaternion):
    """
    Convert a quaternion from transforms to a icv msg quaternion

    :param numpy_quaternion: a numpy quaternion
    :type numpy_quaternion: numpy.array
    :return: a icv quaternion
    :rtype: geometry_msgs.msg.Quaternion
    """
    icv_quaternion = Quaternion()
    icv_quaternion.x = numpy_quaternion[0]
    icv_quaternion.y = numpy_quaternion[1]
    icv_quaternion.z = numpy_quaternion[2]
    icv_quaternion.w = numpy_quaternion[3]
    return icv_quaternion


def carla_rotation_to_RPY(carla_rotation):
    """
    Convert a carla rotation to a roll, pitch, yaw tuple

    Considers the conversion from left-handed system (unreal) to right-handed
    system (icv).
    Considers the conversion from degrees (carla) to radians (icv).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a tuple with 3 elements (roll, pitch, yaw)
    :rtype: tuple
    """
    roll = math.radians(carla_rotation.roll)
    pitch = -math.radians(carla_rotation.pitch)
    yaw = -math.radians(carla_rotation.yaw)

    return (roll, pitch, yaw)


def carla_rotation_to_numpy_quaternion(carla_rotation):
    """
    Convert a carla rotation to a numpy quaternion

    Considers the conversion from left-handed system (unreal) to right-handed
    system (icv).
    Considers the conversion from degrees (carla) to radians (icv).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a numpy.array with 4 elements (quaternion)
    :rtype: numpy.array
    """
    roll, pitch, yaw = carla_rotation_to_RPY(carla_rotation)
    quat = transformations.quaternion_from_euler(roll, pitch, yaw)

    return quat


def carla_rotation_to_icv_quaternion(carla_rotation):
    """
    Convert a carla rotation to a icv quaternion

    Considers the conversion from left-handed system (unreal) to right-handed
    system (icv).
    Considers the conversion from degrees (carla) to radians (icv).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a icv quaternion
    :rtype: geometry_msgs.msg.Quaternion
    """
    quat = carla_rotation_to_numpy_quaternion(carla_rotation)
    icv_quaternion = numpy_quaternion_to_icv_quaternion(quat)

    return icv_quaternion


def carla_rotation_to_numpy_rotation_matrix(carla_rotation):
    """
    Convert a carla rotation to a icv quaternion

    Considers the conversion from left-handed system (unreal) to right-handed
    system (icv).
    Considers the conversion from degrees (carla) to radians (icv).

    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a numpy.array with 3x3 elements
    :rtype: numpy.array
    """
    roll, pitch, yaw = carla_rotation_to_RPY(carla_rotation)
    numpy_array = transformations.euler_matrix(roll, pitch, yaw)
    rotation_matrix = numpy_array[:3, :3]
    return rotation_matrix


def carla_rotation_to_directional_numpy_vector(carla_rotation):
    """
    Convert a carla rotation (as orientation) into a numpy directional vector

    icv_quaternion = np_quaternion_to_icv_quaternion(quat)
    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a numpy.array with 3 elements as directional vector
        representation of the orientation
    :rtype: numpy.array
    """
    rotation_matrix = carla_rotation_to_numpy_rotation_matrix(carla_rotation)
    directional_vector = numpy.array([1, 0, 0])
    rotated_directional_vector = rotation_matrix.dot(directional_vector)
    return rotated_directional_vector


def carla_vector_to_icv_vector_rotated(carla_vector, carla_rotation):
    """
    Rotate carla vector, return it as icv vector

    :param carla_vector: the carla vector
    :type carla_vector: carla.Vector3D
    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: rotated icv vector
    :rtype: Vector3
    """
    rotation_matrix = carla_rotation_to_numpy_rotation_matrix(carla_rotation)
    tmp_array = rotation_matrix.dot(numpy.array([carla_vector.x, carla_vector.y, carla_vector.z]))
    icv_vector = Vector3()
    icv_vector.x = tmp_array[0]
    icv_vector.y = -tmp_array[1]
    icv_vector.z = tmp_array[2]
    return icv_vector


def carla_velocity_to_icv_twist(carla_linear_velocity, carla_angular_velocity, carla_rotation):
    """
    Convert a carla velocity to a icv twist

    Considers the conversion from left-handed system (unreal) to right-handed
    system (icv).

    :param carla_velocity: the carla velocity
    :type carla_velocity: carla.Vector3D
    :param carla_angular_velocity: the carla angular velocity
    :type carla_angular_velocity: carla.Vector3D
    :param carla_rotation: the carla rotation
    :type carla_rotation: carla.Rotation
    :return: a icv twist (with rotation)
    :rtype: geometry_msgs.msg.Twist
    """
    icv_twist = Twist()
    icv_twist.linear = carla_vector_to_icv_vector_rotated(carla_linear_velocity, carla_rotation)
    icv_twist.angular.x = math.radians(carla_angular_velocity.x)
    icv_twist.angular.y = -math.radians(carla_angular_velocity.y)
    icv_twist.angular.z = -math.radians(carla_angular_velocity.z)
    return icv_twist


def carla_velocity_to_numpy_vector(carla_velocity):
    """
    Convert a carla velocity to a numpy array

    Considers the conversion from left-handed system (unreal) to right-handed
    system (icv)

    :param carla_velocity: the carla velocity
    :type carla_velocity: carla.Vector3D
    :return: a numpy.array with 3 elements
    :rtype: numpy.array
    """
    return numpy.array([
        carla_velocity.x,
        -carla_velocity.y,
        carla_velocity.z
    ])


def carla_acceleration_to_icv_accel(carla_acceleration):
    """
    Convert a carla acceleration to a icv accel

    Considers the conversion from left-handed system (unreal) to right-handed
    system (icv)
    The angular accelerations remain zero.

    :param carla_acceleration: the carla acceleration
    :type carla_acceleration: carla.Vector3D
    :return: a icv accel
    :rtype: geometry_msgs.msg.Accel
    """
    icv_accel = Accel()
    icv_accel.linear.x = carla_acceleration.x
    icv_accel.linear.y = -carla_acceleration.y
    icv_accel.linear.z = carla_acceleration.z

    return icv_accel


def carla_transform_to_icv_transform(carla_transform):
    """
    Convert a carla transform to a icv transform

    See carla_location_to_icv_vector3() and carla_rotation_to_icv_quaternion() for details

    :param carla_transform: the carla transform
    :type carla_transform: carla.Transform
    :return: a icv transform
    :rtype: geometry_msgs.msg.Transform
    """
    icv_transform = Transform()

    icv_transform.translation = carla_location_to_icv_vector3(
        carla_transform.location)
    icv_transform.rotation = carla_rotation_to_icv_quaternion(
        carla_transform.rotation)

    return icv_transform


def carla_transform_to_icv_pose(carla_transform):
    """
    Convert a carla transform to a icv pose

    See carla_location_to_icv_point() and carla_rotation_to_icv_quaternion() for details

    :param carla_transform: the carla transform
    :type carla_transform: carla.Transform
    :return: a icv pose
    :rtype: geometry_msgs.msg.Pose
    """
    icv_pose = Pose()

    icv_pose.position = carla_location_to_icv_point(
        carla_transform.location)
    icv_pose.orientation = carla_rotation_to_icv_quaternion(
        carla_transform.rotation)

    return icv_pose


def carla_location_to_pose(carla_location):
    """
    Convert a carla location to a icv pose

    See carla_location_to_icv_point() for details.
    pose quaternion remains zero.

    :param carla_location: the carla location
    :type carla_location: carla.Location
    :return: a icv pose
    :rtype: geometry_msgs.msg.Pose
    """
    icv_pose = Pose()
    icv_pose.position = carla_location_to_icv_point(carla_location)
    return icv_pose
