# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: robot.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import builder as _builder
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0brobot.proto\x12\x05robot\"\xa9\x01\n\x05Robot\x12\x14\n\x0cjoint_angles\x18\x01 \x03(\x01\x12\x18\n\x10joint_velocities\x18\x02 \x03(\x01\x12\x10\n\x08tcp_pose\x18\x03 \x03(\x01\x12\x18\n\x10gripper_position\x18\x04 \x01(\x01\x12\x15\n\rgripper_force\x18\x05 \x01(\x01\x12\x13\n\x0b\x63olor_image\x18\x06 \x01(\x0c\x12\x18\n\x10\x63olor_extrinsics\x18\x07 \x03(\x01\x62\x06proto3')

_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'robot_pb2', globals())
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _ROBOT._serialized_start=23
  _ROBOT._serialized_end=192
# @@protoc_insertion_point(module_scope)
