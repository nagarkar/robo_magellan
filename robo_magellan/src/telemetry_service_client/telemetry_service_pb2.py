# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: telemetry_service.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf.internal import enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='telemetry_service.proto',
  package='telemetry',
  syntax='proto3',
  serialized_pb=_b('\n\x17telemetry_service.proto\x12\ttelemetry\"\x12\n\x10\x41ttitudeQRequest\"V\n\x11\x41ttitudeQResponse\x12\n\n\x02Qc\x18\x01 \x01(\x02\x12\n\n\x02Qx\x18\x02 \x01(\x02\x12\n\n\x02Qy\x18\x03 \x01(\x02\x12\n\n\x02Qz\x18\x04 \x01(\x02\x12\x11\n\ttimestamp\x18\x05 \x01(\x04\"/\n\x08HitEvent\x12\x11\n\ttimestamp\x18\x01 \x01(\x04\x12\x10\n\x08priority\x18\x02 \x01(\x05\"*\n\x0cHeadingAngle\x12\x0b\n\x03yaw\x18\x01 \x01(\x02\x12\r\n\x05pitch\x18\x02 \x01(\x02\"S\n\tEventBase\x12!\n\x06source\x18\x01 \x01(\x0e\x32\x11.telemetry.Source\x12\x10\n\x08priority\x18\x02 \x01(\x05\x12\x11\n\ttimestamp\x18\x03 \x01(\x04\"E\n\rEventResponse\x12!\n\x06source\x18\x01 \x01(\x0e\x32\x11.telemetry.Source\x12\x11\n\ttimestamp\x18\x02 \x01(\x04\"{\n\x0cHeadingEvent\x12\"\n\x04\x62\x61se\x18\x01 \x01(\x0b\x32\x14.telemetry.EventBase\x12\x35\n\x14relativeAngleRadians\x18\x02 \x01(\x0b\x32\x17.telemetry.HeadingAngle\x12\x10\n\x08\x64istance\x18\x03 \x01(\x02\"/\n\x08Waypoint\x12\x11\n\tlongitude\x18\x01 \x01(\x02\x12\x10\n\x08latitude\x18\x02 \x01(\x02\"2\n\tWaypoints\x12%\n\x08\x65lements\x18\x01 \x03(\x0b\x32\x13.telemetry.Waypoint\"\x0b\n\tNoContent*<\n\x08Priority\x12\x14\n\x10UNKNOWN_PRIORITY\x10\x00\x12\x08\n\x04HIGH\x10\x01\x12\x07\n\x03MED\x10\x32\x12\x07\n\x03LOW\x10\x64*d\n\x06Source\x12\x12\n\x0eUNKNOWN_SOURCE\x10\x00\x12\x15\n\x11RGBD_CAMERA_FRONT\x10\x01\x12\x17\n\x13HCSR04_SENSOR_FRONT\x10\x02\x12\x16\n\x12\x44\x45PTH_CAMERA_FRONT\x10\x03\x32\xad\x03\n\x10TelemetryService\x12K\n\x0cGetAttitudeQ\x12\x1b.telemetry.AttitudeQRequest\x1a\x1c.telemetry.AttitudeQResponse\"\x00\x12R\n\x11GetAttitudeStream\x12\x1b.telemetry.AttitudeQRequest\x1a\x1c.telemetry.AttitudeQResponse\"\x00\x30\x01\x12;\n\x08hitEvent\x12\x13.telemetry.HitEvent\x1a\x18.telemetry.EventResponse\"\x00\x12\x43\n\x0cheadingEvent\x12\x17.telemetry.HeadingEvent\x1a\x18.telemetry.EventResponse\"\x00\x12\x38\n\tsetTarget\x12\x13.telemetry.Waypoint\x1a\x14.telemetry.NoContent\"\x00\x12<\n\x0csetWaypoints\x12\x14.telemetry.Waypoints\x1a\x14.telemetry.NoContent\"\x00\x42*\n\x14ppcbs.grpc.telemetryB\x10TelemetryServiceP\x01\x62\x06proto3')
)

_PRIORITY = _descriptor.EnumDescriptor(
  name='Priority',
  full_name='telemetry.Priority',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN_PRIORITY', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='HIGH', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='MED', index=2, number=50,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LOW', index=3, number=100,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=634,
  serialized_end=694,
)
_sym_db.RegisterEnumDescriptor(_PRIORITY)

Priority = enum_type_wrapper.EnumTypeWrapper(_PRIORITY)
_SOURCE = _descriptor.EnumDescriptor(
  name='Source',
  full_name='telemetry.Source',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN_SOURCE', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='RGBD_CAMERA_FRONT', index=1, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='HCSR04_SENSOR_FRONT', index=2, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='DEPTH_CAMERA_FRONT', index=3, number=3,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=696,
  serialized_end=796,
)
_sym_db.RegisterEnumDescriptor(_SOURCE)

Source = enum_type_wrapper.EnumTypeWrapper(_SOURCE)
UNKNOWN_PRIORITY = 0
HIGH = 1
MED = 50
LOW = 100
UNKNOWN_SOURCE = 0
RGBD_CAMERA_FRONT = 1
HCSR04_SENSOR_FRONT = 2
DEPTH_CAMERA_FRONT = 3



_ATTITUDEQREQUEST = _descriptor.Descriptor(
  name='AttitudeQRequest',
  full_name='telemetry.AttitudeQRequest',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=38,
  serialized_end=56,
)


_ATTITUDEQRESPONSE = _descriptor.Descriptor(
  name='AttitudeQResponse',
  full_name='telemetry.AttitudeQResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='Qc', full_name='telemetry.AttitudeQResponse.Qc', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='Qx', full_name='telemetry.AttitudeQResponse.Qx', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='Qy', full_name='telemetry.AttitudeQResponse.Qy', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='Qz', full_name='telemetry.AttitudeQResponse.Qz', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='telemetry.AttitudeQResponse.timestamp', index=4,
      number=5, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=58,
  serialized_end=144,
)


_HITEVENT = _descriptor.Descriptor(
  name='HitEvent',
  full_name='telemetry.HitEvent',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='telemetry.HitEvent.timestamp', index=0,
      number=1, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='priority', full_name='telemetry.HitEvent.priority', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=146,
  serialized_end=193,
)


_HEADINGANGLE = _descriptor.Descriptor(
  name='HeadingAngle',
  full_name='telemetry.HeadingAngle',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='yaw', full_name='telemetry.HeadingAngle.yaw', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pitch', full_name='telemetry.HeadingAngle.pitch', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=195,
  serialized_end=237,
)


_EVENTBASE = _descriptor.Descriptor(
  name='EventBase',
  full_name='telemetry.EventBase',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='source', full_name='telemetry.EventBase.source', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='priority', full_name='telemetry.EventBase.priority', index=1,
      number=2, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='telemetry.EventBase.timestamp', index=2,
      number=3, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=239,
  serialized_end=322,
)


_EVENTRESPONSE = _descriptor.Descriptor(
  name='EventResponse',
  full_name='telemetry.EventResponse',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='source', full_name='telemetry.EventResponse.source', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='timestamp', full_name='telemetry.EventResponse.timestamp', index=1,
      number=2, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=324,
  serialized_end=393,
)


_HEADINGEVENT = _descriptor.Descriptor(
  name='HeadingEvent',
  full_name='telemetry.HeadingEvent',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='base', full_name='telemetry.HeadingEvent.base', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='relativeAngleRadians', full_name='telemetry.HeadingEvent.relativeAngleRadians', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='distance', full_name='telemetry.HeadingEvent.distance', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=395,
  serialized_end=518,
)


_WAYPOINT = _descriptor.Descriptor(
  name='Waypoint',
  full_name='telemetry.Waypoint',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='longitude', full_name='telemetry.Waypoint.longitude', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='latitude', full_name='telemetry.Waypoint.latitude', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=520,
  serialized_end=567,
)


_WAYPOINTS = _descriptor.Descriptor(
  name='Waypoints',
  full_name='telemetry.Waypoints',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='elements', full_name='telemetry.Waypoints.elements', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=569,
  serialized_end=619,
)


_NOCONTENT = _descriptor.Descriptor(
  name='NoContent',
  full_name='telemetry.NoContent',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=621,
  serialized_end=632,
)

_EVENTBASE.fields_by_name['source'].enum_type = _SOURCE
_EVENTRESPONSE.fields_by_name['source'].enum_type = _SOURCE
_HEADINGEVENT.fields_by_name['base'].message_type = _EVENTBASE
_HEADINGEVENT.fields_by_name['relativeAngleRadians'].message_type = _HEADINGANGLE
_WAYPOINTS.fields_by_name['elements'].message_type = _WAYPOINT
DESCRIPTOR.message_types_by_name['AttitudeQRequest'] = _ATTITUDEQREQUEST
DESCRIPTOR.message_types_by_name['AttitudeQResponse'] = _ATTITUDEQRESPONSE
DESCRIPTOR.message_types_by_name['HitEvent'] = _HITEVENT
DESCRIPTOR.message_types_by_name['HeadingAngle'] = _HEADINGANGLE
DESCRIPTOR.message_types_by_name['EventBase'] = _EVENTBASE
DESCRIPTOR.message_types_by_name['EventResponse'] = _EVENTRESPONSE
DESCRIPTOR.message_types_by_name['HeadingEvent'] = _HEADINGEVENT
DESCRIPTOR.message_types_by_name['Waypoint'] = _WAYPOINT
DESCRIPTOR.message_types_by_name['Waypoints'] = _WAYPOINTS
DESCRIPTOR.message_types_by_name['NoContent'] = _NOCONTENT
DESCRIPTOR.enum_types_by_name['Priority'] = _PRIORITY
DESCRIPTOR.enum_types_by_name['Source'] = _SOURCE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

AttitudeQRequest = _reflection.GeneratedProtocolMessageType('AttitudeQRequest', (_message.Message,), dict(
  DESCRIPTOR = _ATTITUDEQREQUEST,
  __module__ = 'telemetry_service_pb2'
  # @@protoc_insertion_point(class_scope:telemetry.AttitudeQRequest)
  ))
_sym_db.RegisterMessage(AttitudeQRequest)

AttitudeQResponse = _reflection.GeneratedProtocolMessageType('AttitudeQResponse', (_message.Message,), dict(
  DESCRIPTOR = _ATTITUDEQRESPONSE,
  __module__ = 'telemetry_service_pb2'
  # @@protoc_insertion_point(class_scope:telemetry.AttitudeQResponse)
  ))
_sym_db.RegisterMessage(AttitudeQResponse)

HitEvent = _reflection.GeneratedProtocolMessageType('HitEvent', (_message.Message,), dict(
  DESCRIPTOR = _HITEVENT,
  __module__ = 'telemetry_service_pb2'
  # @@protoc_insertion_point(class_scope:telemetry.HitEvent)
  ))
_sym_db.RegisterMessage(HitEvent)

HeadingAngle = _reflection.GeneratedProtocolMessageType('HeadingAngle', (_message.Message,), dict(
  DESCRIPTOR = _HEADINGANGLE,
  __module__ = 'telemetry_service_pb2'
  # @@protoc_insertion_point(class_scope:telemetry.HeadingAngle)
  ))
_sym_db.RegisterMessage(HeadingAngle)

EventBase = _reflection.GeneratedProtocolMessageType('EventBase', (_message.Message,), dict(
  DESCRIPTOR = _EVENTBASE,
  __module__ = 'telemetry_service_pb2'
  # @@protoc_insertion_point(class_scope:telemetry.EventBase)
  ))
_sym_db.RegisterMessage(EventBase)

EventResponse = _reflection.GeneratedProtocolMessageType('EventResponse', (_message.Message,), dict(
  DESCRIPTOR = _EVENTRESPONSE,
  __module__ = 'telemetry_service_pb2'
  # @@protoc_insertion_point(class_scope:telemetry.EventResponse)
  ))
_sym_db.RegisterMessage(EventResponse)

HeadingEvent = _reflection.GeneratedProtocolMessageType('HeadingEvent', (_message.Message,), dict(
  DESCRIPTOR = _HEADINGEVENT,
  __module__ = 'telemetry_service_pb2'
  # @@protoc_insertion_point(class_scope:telemetry.HeadingEvent)
  ))
_sym_db.RegisterMessage(HeadingEvent)

Waypoint = _reflection.GeneratedProtocolMessageType('Waypoint', (_message.Message,), dict(
  DESCRIPTOR = _WAYPOINT,
  __module__ = 'telemetry_service_pb2'
  # @@protoc_insertion_point(class_scope:telemetry.Waypoint)
  ))
_sym_db.RegisterMessage(Waypoint)

Waypoints = _reflection.GeneratedProtocolMessageType('Waypoints', (_message.Message,), dict(
  DESCRIPTOR = _WAYPOINTS,
  __module__ = 'telemetry_service_pb2'
  # @@protoc_insertion_point(class_scope:telemetry.Waypoints)
  ))
_sym_db.RegisterMessage(Waypoints)

NoContent = _reflection.GeneratedProtocolMessageType('NoContent', (_message.Message,), dict(
  DESCRIPTOR = _NOCONTENT,
  __module__ = 'telemetry_service_pb2'
  # @@protoc_insertion_point(class_scope:telemetry.NoContent)
  ))
_sym_db.RegisterMessage(NoContent)


DESCRIPTOR.has_options = True
DESCRIPTOR._options = _descriptor._ParseOptions(descriptor_pb2.FileOptions(), _b('\n\024ppcbs.grpc.telemetryB\020TelemetryServiceP\001'))

_TELEMETRYSERVICE = _descriptor.ServiceDescriptor(
  name='TelemetryService',
  full_name='telemetry.TelemetryService',
  file=DESCRIPTOR,
  index=0,
  options=None,
  serialized_start=799,
  serialized_end=1228,
  methods=[
  _descriptor.MethodDescriptor(
    name='GetAttitudeQ',
    full_name='telemetry.TelemetryService.GetAttitudeQ',
    index=0,
    containing_service=None,
    input_type=_ATTITUDEQREQUEST,
    output_type=_ATTITUDEQRESPONSE,
    options=None,
  ),
  _descriptor.MethodDescriptor(
    name='GetAttitudeStream',
    full_name='telemetry.TelemetryService.GetAttitudeStream',
    index=1,
    containing_service=None,
    input_type=_ATTITUDEQREQUEST,
    output_type=_ATTITUDEQRESPONSE,
    options=None,
  ),
  _descriptor.MethodDescriptor(
    name='hitEvent',
    full_name='telemetry.TelemetryService.hitEvent',
    index=2,
    containing_service=None,
    input_type=_HITEVENT,
    output_type=_EVENTRESPONSE,
    options=None,
  ),
  _descriptor.MethodDescriptor(
    name='headingEvent',
    full_name='telemetry.TelemetryService.headingEvent',
    index=3,
    containing_service=None,
    input_type=_HEADINGEVENT,
    output_type=_EVENTRESPONSE,
    options=None,
  ),
  _descriptor.MethodDescriptor(
    name='setTarget',
    full_name='telemetry.TelemetryService.setTarget',
    index=4,
    containing_service=None,
    input_type=_WAYPOINT,
    output_type=_NOCONTENT,
    options=None,
  ),
  _descriptor.MethodDescriptor(
    name='setWaypoints',
    full_name='telemetry.TelemetryService.setWaypoints',
    index=5,
    containing_service=None,
    input_type=_WAYPOINTS,
    output_type=_NOCONTENT,
    options=None,
  ),
])
_sym_db.RegisterServiceDescriptor(_TELEMETRYSERVICE)

DESCRIPTOR.services_by_name['TelemetryService'] = _TELEMETRYSERVICE

# @@protoc_insertion_point(module_scope)
