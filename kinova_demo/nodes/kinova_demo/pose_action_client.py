#! /usr/bin/env python
"""A helper program to test cartesian goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy

import sys
import numpy as np

import actionlib
import kinova_msgs.msg
import std_msgs.msg
import geometry_msgs.msg


import goal_generators

import math
import argparse

""" Global variable """
numJoint = 0
numFinger = 0
prefix = 'NO_ROBOT_TYPE_DEFINED'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger


def cartesian_pose_client(position, orientation):
    """Send a cartesian goal to the action server."""
    action_address = '/' + prefix + '_arm_driver/pose_action/tool_pose'
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + '_api_origin'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    print('goal.pose in main 1: {}'.format(goal.pose.pose)) # debug

    client.send_goal(goal)

    if client.wait_for_result(rospy.Duration(10.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None


def QuaternionNorm(Q_raw):
    qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
    qx_ = qx_temp/qnorm
    qy_ = qy_temp/qnorm
    qz_ = qz_temp/qnorm
    qw_ = qw_temp/qnorm
    Q_normed_ = [qx_, qy_, qz_, qw_]
    return Q_normed_


def Quaternion2EulerXYZ(Q_raw):
    Q_normed = QuaternionNorm(Q_raw)
    qx_ = Q_normed[0]
    qy_ = Q_normed[1]
    qz_ = Q_normed[2]
    qw_ = Q_normed[3]

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]
    return EulerXYZ_


def EulerXYZ2Quaternion(EulerXYZ_):
    tx_, ty_, tz_ = EulerXYZ_[0:3]
    sx = math.sin(0.5 * tx_)
    cx = math.cos(0.5 * tx_)
    sy = math.sin(0.5 * ty_)
    cy = math.cos(0.5 * ty_)
    sz = math.sin(0.5 * tz_)
    cz = math.cos(0.5 * tz_)

    qx_ = sx * cy * cz + cx * sy * sz
    qy_ = -sx * cy * sz + cx * sy * cz
    qz_ = sx * sy * cz + cx * cy * sz
    qw_ = -sx * sy * sz + cx * cy * cz

    Q_ = [qx_, qy_, qz_, qw_]
    return Q_


def argumentParser(argument_):
    """ Argument parser """
    parser = argparse.ArgumentParser(description='Drive robot end-effector to command Cartesian pose')
    parser.add_argument('robotType', metavar='robotType', type=int, choices=range(7),
                        help='Index for robotType: JACOV1_ASSISTIVE_3FINGERS = 0, MICO_6DOF_SERVICE_2FINGERS = 1, MICO_4DOF_SERVICE_2FINGERS = 2, JACOV2_6DOF_SERVICE_3FINGERS = 3, JACOV2_4DOF_SERVICE_3FINGERS = 4, MICO_6DOF_ASSISTIVE_2FINGER2 = 5, JACOV2_6DOF_ASSISTIVE_3FINGERS = 6')
    parser.add_argument('unit', metavar='unit', type=str, nargs='?', default='mq',
                        choices={'mq', 'mdeg', 'mrad'},
                        help='Unit of Cartesian pose command, in mq(Position meter, Orientation Quaternion),  mdeg(Position meter, Orientation Euler-XYZ in degree), mrad(Position meter, Orientation Euler-XYZ in radian)]')
    parser.add_argument('pose_value', nargs='*', type=float, help='Cartesian pose values: first three values for position, and last three(unit mdeg or mrad)/four(unit mq) for Orientation')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='display Cartesian pose values in alternative convention(mq, mdeg or mrad)')
    # parser.add_argument('-f', action='store_true', help='assign finger values from a file')

    args_ = parser.parse_args(argument_)
    print('pose_mq in main 1: {}'.format(args_.pose_value))  # debug
    return args_


def robotTypeParser(robotType_):
    """ Argument robotType """
    global numJoint, numFinger, prefix, finger_maxDist, finger_maxTurn
    if robotType_ == 0:
        numJoint = 6
        numFinger = 3
        # prefix = 'j16a3'
        prefix = 'jaco'
    elif robotType_ == 1:
        numJoint = 6
        numFinger = 2
        # prefix = 'm16s2'
        prefix = 'mico'
    elif robotType_ == 2:
        numJoint = 4
        numFinger = 2
        # prefix = 'm14s2'
        prefix = 'mico'
    elif robotType_ == 3:
        numJoint = 6
        numFinger = 3
        # prefix = 'j26s3'
        prefix = 'jaco'
    elif robotType_ == 4:
        numJoint = 4
        numFinger = 3
        # prefix = 'j24s3'
        prefix = 'jaco'
    elif robotType_ == 5:
        numJoint = 6
        numFinger = 2
        finger_maxDist = 18.9/2/1000  # max distance for one finger in meter
        finger_maxTurn = 6800  # max thread turn for one finger
        # prefix = 'm16a2' # refefine robotType m6a2-->mico-6DOF-assistive-2Fingers
        prefix = 'mico'
    elif robotType_ == 6:
        numJoint = 6
        numFinger = 3
        # prefix = 'j26a3'
        prefix = 'jaco'
    else:
        raise Exception('Undefined robotType: {}'.format(robotType_))


def unitParser(unit_, pose_value_):
    """ Argument unit """
    position_ = pose_value_[:3]
    orientation_ = pose_value_[3:]

    print('pose_value_ in unitParser 1: {}'.format(pose_value_))  # debug

    if unit_ == 'mq':
        orientation_q = orientation_
        orientation_rad = Quaternion2EulerXYZ(orientation_q)
        orientation_deg = list(map(math.degrees, orientation_rad))
    elif unit_ == 'mdeg':
        orientation_deg = orientation_
        orientation_rad = list(map(math.radians, orientation_deg))
        orientation_q = EulerXYZ2Quaternion(orientation_rad)
    elif unit_ == 'mrad':
        orientation_rad = orientation_
        orientation_deg = list(map(math.degrees, orientation_rad))
        orientation_q = EulerXYZ2Quaternion(orientation_rad)
    else:
        raise Exception("Finger value have to be in mq, mdeg or mrad")

    pose_mq_ = position_ + orientation_q
    pose_mdeg_ = position_ + orientation_deg
    pose_mrad_ = position_ + orientation_rad

    print('pose_mq in unitParser 1: {}'.format(pose_mq_))  # debug

    return pose_mq_, pose_mdeg_, pose_mrad_


def verboseParser(verbose, pose_mq_):
    """ Argument verbose """
    position_ = pose_mq_[:3]
    orientation_q = pose_mq_[3:]
    if verbose:
        orientation_rad = Quaternion2EulerXYZ(orientation_q)
        orientation_deg = list(map(math.degrees, orientation_rad))
        print('Cartesian position is: {}'.format(position_))
        print('Cartesian orientation in Quaternion is: ')
        print('qx, qy, qz, qw: {:0.3f}'.format(orientation_q))
        print('Cartesian orientation in Euler-XYZ(radian) is: ')
        print('tx, ty, tz: {:0.3f}'.format(orientation_rad))
        print('Cartesian orientation in Euler-XYZ(degree) is: ')
        print('tx, ty, tz: {:3.1f}'.format(orientation_deg))


if __name__ == '__main__':

    args = argumentParser(None)

    robotTypeParser(args.robotType)

    if args.unit == 'mq':
        if len(args.pose_value) != 7:
            print('Number of input values {} is not equal to 7 (3 position + 4 Quaternion).'.format(len(args.pose_value)))
            sys.exit(0)
    elif (args.unit == 'mrad') | (args.unit == 'mdeg'):
        if len(args.pose_value) != 6:
            print('Number of input values {} is not equal to 6(3 position + 3 EulerAngles).'.format(len(args.pose_value)))
            sys.exit(0)
    else:
        raise Exception('Finger value have to be in mq, mdeg or mrad')


    pose_mq, pose_mdeg, pose_mrad = unitParser(args.unit, args.pose_value)

    print('pose_mq in main 1: {}'.format(pose_mq)) # debug

    try:
        rospy.init_node(prefix + '_pose_action_client')

        poses = [float(n) for n in pose_mq]

        result = cartesian_pose_client(poses[:3], poses[3:])

        print('Cartesian pose sent!')

    except rospy.ROSInterruptException:
        print "program interrupted before completion"


    verboseParser(args.verbose, pose_mq)