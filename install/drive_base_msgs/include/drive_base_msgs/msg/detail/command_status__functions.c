// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from drive_base_msgs:msg/CommandStatus.idl
// generated code does not contain a copyright notice
#include "drive_base_msgs/msg/detail/command_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `cmd_header`
#include "drive_base_msgs/msg/detail/command_header__functions.h"

bool
drive_base_msgs__msg__CommandStatus__init(drive_base_msgs__msg__CommandStatus * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    drive_base_msgs__msg__CommandStatus__fini(msg);
    return false;
  }
  // cmd_header
  if (!drive_base_msgs__msg__CommandHeader__init(&msg->cmd_header)) {
    drive_base_msgs__msg__CommandStatus__fini(msg);
    return false;
  }
  // status
  return true;
}

void
drive_base_msgs__msg__CommandStatus__fini(drive_base_msgs__msg__CommandStatus * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // cmd_header
  drive_base_msgs__msg__CommandHeader__fini(&msg->cmd_header);
  // status
}

bool
drive_base_msgs__msg__CommandStatus__are_equal(const drive_base_msgs__msg__CommandStatus * lhs, const drive_base_msgs__msg__CommandStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  // cmd_header
  if (!drive_base_msgs__msg__CommandHeader__are_equal(
      &(lhs->cmd_header), &(rhs->cmd_header)))
  {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  return true;
}

bool
drive_base_msgs__msg__CommandStatus__copy(
  const drive_base_msgs__msg__CommandStatus * input,
  drive_base_msgs__msg__CommandStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  // cmd_header
  if (!drive_base_msgs__msg__CommandHeader__copy(
      &(input->cmd_header), &(output->cmd_header)))
  {
    return false;
  }
  // status
  output->status = input->status;
  return true;
}

drive_base_msgs__msg__CommandStatus *
drive_base_msgs__msg__CommandStatus__create()
{
  drive_base_msgs__msg__CommandStatus * msg = (drive_base_msgs__msg__CommandStatus *)malloc(sizeof(drive_base_msgs__msg__CommandStatus));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(drive_base_msgs__msg__CommandStatus));
  bool success = drive_base_msgs__msg__CommandStatus__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
drive_base_msgs__msg__CommandStatus__destroy(drive_base_msgs__msg__CommandStatus * msg)
{
  if (msg) {
    drive_base_msgs__msg__CommandStatus__fini(msg);
  }
  free(msg);
}


bool
drive_base_msgs__msg__CommandStatus__Sequence__init(drive_base_msgs__msg__CommandStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  drive_base_msgs__msg__CommandStatus * data = NULL;
  if (size) {
    data = (drive_base_msgs__msg__CommandStatus *)calloc(size, sizeof(drive_base_msgs__msg__CommandStatus));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = drive_base_msgs__msg__CommandStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        drive_base_msgs__msg__CommandStatus__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
drive_base_msgs__msg__CommandStatus__Sequence__fini(drive_base_msgs__msg__CommandStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      drive_base_msgs__msg__CommandStatus__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

drive_base_msgs__msg__CommandStatus__Sequence *
drive_base_msgs__msg__CommandStatus__Sequence__create(size_t size)
{
  drive_base_msgs__msg__CommandStatus__Sequence * array = (drive_base_msgs__msg__CommandStatus__Sequence *)malloc(sizeof(drive_base_msgs__msg__CommandStatus__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = drive_base_msgs__msg__CommandStatus__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
drive_base_msgs__msg__CommandStatus__Sequence__destroy(drive_base_msgs__msg__CommandStatus__Sequence * array)
{
  if (array) {
    drive_base_msgs__msg__CommandStatus__Sequence__fini(array);
  }
  free(array);
}

bool
drive_base_msgs__msg__CommandStatus__Sequence__are_equal(const drive_base_msgs__msg__CommandStatus__Sequence * lhs, const drive_base_msgs__msg__CommandStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!drive_base_msgs__msg__CommandStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
drive_base_msgs__msg__CommandStatus__Sequence__copy(
  const drive_base_msgs__msg__CommandStatus__Sequence * input,
  drive_base_msgs__msg__CommandStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(drive_base_msgs__msg__CommandStatus);
    drive_base_msgs__msg__CommandStatus * data =
      (drive_base_msgs__msg__CommandStatus *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!drive_base_msgs__msg__CommandStatus__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          drive_base_msgs__msg__CommandStatus__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!drive_base_msgs__msg__CommandStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
