// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from torsobot_interfaces:srv/RequestControlParams.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "torsobot_interfaces/srv/detail/request_control_params__functions.h"
#include "torsobot_interfaces/srv/detail/request_control_params__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace torsobot_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _RequestControlParams_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RequestControlParams_Request_type_support_ids_t;

static const _RequestControlParams_Request_type_support_ids_t _RequestControlParams_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _RequestControlParams_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RequestControlParams_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RequestControlParams_Request_type_support_symbol_names_t _RequestControlParams_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, torsobot_interfaces, srv, RequestControlParams_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, torsobot_interfaces, srv, RequestControlParams_Request)),
  }
};

typedef struct _RequestControlParams_Request_type_support_data_t
{
  void * data[2];
} _RequestControlParams_Request_type_support_data_t;

static _RequestControlParams_Request_type_support_data_t _RequestControlParams_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RequestControlParams_Request_message_typesupport_map = {
  2,
  "torsobot_interfaces",
  &_RequestControlParams_Request_message_typesupport_ids.typesupport_identifier[0],
  &_RequestControlParams_Request_message_typesupport_symbol_names.symbol_name[0],
  &_RequestControlParams_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RequestControlParams_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RequestControlParams_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &torsobot_interfaces__srv__RequestControlParams_Request__get_type_hash,
  &torsobot_interfaces__srv__RequestControlParams_Request__get_type_description,
  &torsobot_interfaces__srv__RequestControlParams_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace torsobot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<torsobot_interfaces::srv::RequestControlParams_Request>()
{
  return &::torsobot_interfaces::srv::rosidl_typesupport_cpp::RequestControlParams_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, torsobot_interfaces, srv, RequestControlParams_Request)() {
  return get_message_type_support_handle<torsobot_interfaces::srv::RequestControlParams_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "torsobot_interfaces/srv/detail/request_control_params__functions.h"
// already included above
// #include "torsobot_interfaces/srv/detail/request_control_params__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace torsobot_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _RequestControlParams_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RequestControlParams_Response_type_support_ids_t;

static const _RequestControlParams_Response_type_support_ids_t _RequestControlParams_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _RequestControlParams_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RequestControlParams_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RequestControlParams_Response_type_support_symbol_names_t _RequestControlParams_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, torsobot_interfaces, srv, RequestControlParams_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, torsobot_interfaces, srv, RequestControlParams_Response)),
  }
};

typedef struct _RequestControlParams_Response_type_support_data_t
{
  void * data[2];
} _RequestControlParams_Response_type_support_data_t;

static _RequestControlParams_Response_type_support_data_t _RequestControlParams_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RequestControlParams_Response_message_typesupport_map = {
  2,
  "torsobot_interfaces",
  &_RequestControlParams_Response_message_typesupport_ids.typesupport_identifier[0],
  &_RequestControlParams_Response_message_typesupport_symbol_names.symbol_name[0],
  &_RequestControlParams_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RequestControlParams_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RequestControlParams_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &torsobot_interfaces__srv__RequestControlParams_Response__get_type_hash,
  &torsobot_interfaces__srv__RequestControlParams_Response__get_type_description,
  &torsobot_interfaces__srv__RequestControlParams_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace torsobot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<torsobot_interfaces::srv::RequestControlParams_Response>()
{
  return &::torsobot_interfaces::srv::rosidl_typesupport_cpp::RequestControlParams_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, torsobot_interfaces, srv, RequestControlParams_Response)() {
  return get_message_type_support_handle<torsobot_interfaces::srv::RequestControlParams_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "torsobot_interfaces/srv/detail/request_control_params__functions.h"
// already included above
// #include "torsobot_interfaces/srv/detail/request_control_params__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace torsobot_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _RequestControlParams_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RequestControlParams_Event_type_support_ids_t;

static const _RequestControlParams_Event_type_support_ids_t _RequestControlParams_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _RequestControlParams_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RequestControlParams_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RequestControlParams_Event_type_support_symbol_names_t _RequestControlParams_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, torsobot_interfaces, srv, RequestControlParams_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, torsobot_interfaces, srv, RequestControlParams_Event)),
  }
};

typedef struct _RequestControlParams_Event_type_support_data_t
{
  void * data[2];
} _RequestControlParams_Event_type_support_data_t;

static _RequestControlParams_Event_type_support_data_t _RequestControlParams_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RequestControlParams_Event_message_typesupport_map = {
  2,
  "torsobot_interfaces",
  &_RequestControlParams_Event_message_typesupport_ids.typesupport_identifier[0],
  &_RequestControlParams_Event_message_typesupport_symbol_names.symbol_name[0],
  &_RequestControlParams_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t RequestControlParams_Event_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RequestControlParams_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &torsobot_interfaces__srv__RequestControlParams_Event__get_type_hash,
  &torsobot_interfaces__srv__RequestControlParams_Event__get_type_description,
  &torsobot_interfaces__srv__RequestControlParams_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace torsobot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<torsobot_interfaces::srv::RequestControlParams_Event>()
{
  return &::torsobot_interfaces::srv::rosidl_typesupport_cpp::RequestControlParams_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, torsobot_interfaces, srv, RequestControlParams_Event)() {
  return get_message_type_support_handle<torsobot_interfaces::srv::RequestControlParams_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "torsobot_interfaces/srv/detail/request_control_params__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace torsobot_interfaces
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _RequestControlParams_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _RequestControlParams_type_support_ids_t;

static const _RequestControlParams_type_support_ids_t _RequestControlParams_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _RequestControlParams_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _RequestControlParams_type_support_symbol_names_t;
#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _RequestControlParams_type_support_symbol_names_t _RequestControlParams_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, torsobot_interfaces, srv, RequestControlParams)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, torsobot_interfaces, srv, RequestControlParams)),
  }
};

typedef struct _RequestControlParams_type_support_data_t
{
  void * data[2];
} _RequestControlParams_type_support_data_t;

static _RequestControlParams_type_support_data_t _RequestControlParams_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _RequestControlParams_service_typesupport_map = {
  2,
  "torsobot_interfaces",
  &_RequestControlParams_service_typesupport_ids.typesupport_identifier[0],
  &_RequestControlParams_service_typesupport_symbol_names.symbol_name[0],
  &_RequestControlParams_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t RequestControlParams_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_RequestControlParams_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<torsobot_interfaces::srv::RequestControlParams_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<torsobot_interfaces::srv::RequestControlParams_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<torsobot_interfaces::srv::RequestControlParams_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<torsobot_interfaces::srv::RequestControlParams>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<torsobot_interfaces::srv::RequestControlParams>,
  &torsobot_interfaces__srv__RequestControlParams__get_type_hash,
  &torsobot_interfaces__srv__RequestControlParams__get_type_description,
  &torsobot_interfaces__srv__RequestControlParams__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace torsobot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<torsobot_interfaces::srv::RequestControlParams>()
{
  return &::torsobot_interfaces::srv::rosidl_typesupport_cpp::RequestControlParams_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, torsobot_interfaces, srv, RequestControlParams)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<torsobot_interfaces::srv::RequestControlParams>();
}

#ifdef __cplusplus
}
#endif
