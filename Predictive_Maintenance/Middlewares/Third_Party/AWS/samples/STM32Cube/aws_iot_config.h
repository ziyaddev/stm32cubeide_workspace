/*
 * Copyright 2010-2015 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

/**
 * NOTICE: THIS FILE WAS HEAVILY MODIFIED BY STMICROELECTRONICS, INC.
 */

/**
 * @file aws_iot_config.h
 * @brief AWS IoT specific configuration file
 */

#ifndef SRC_SHADOW_IOT_SHADOW_CONFIG_H_
#define SRC_SHADOW_IOT_SHADOW_CONFIG_H_

// =================================================
/* The following definitions are read the console and stored in the user configuration section of the Flash memory.
 * AWS_IOT_MQTT_HOST, AWS_IOT_MQTT_CLIENT_ID/AWS_IOT_MY_THING_NAME
 */

/* AWS_IOT_MQTT_HOST must be of the form
   "xxxxxxxxxxxx.iot.xxxxxxxxx.amazonaws.com"
   without the https. */
// #define AWS_IOT_MQTT_HOST              ""
#define AWS_IOT_MQTT_PORT              8883

/* AWS_IOT_MQTT_CLIENT_ID must be of the form "xxxxxxxxxxxx". */
// #define AWS_IOT_MQTT_CLIENT_ID         ""

/* AWS_IOT_MY_THING_NAME must be of the form "xxxxxxxxxxxx". */
// #define AWS_IOT_MY_THING0_NAME          ""
// =================================================

// MQTT PubSub
#define AWS_IOT_MQTT_TX_BUF_LEN 1000 ///< Any time a message is sent out through the MQTT layer. The message is copied into this buffer anytime a publish is done. This will also be used in the case of Thing Shadow
#define AWS_IOT_MQTT_RX_BUF_LEN 1000 ///< Any message that comes into the device should be less than this buffer size. If a received message is bigger than this buffer size the message will be dropped.
#define AWS_IOT_MQTT_NUM_SUBSCRIBE_HANDLERS 6 ///< Maximum number of topic filters the MQTT client can handle at any given time. This should be increased appropriately when using Thing Shadow

// Thing Shadow specific configs
#define SHADOW_MAX_SIZE_OF_RX_BUFFER AWS_IOT_MQTT_RX_BUF_LEN+1 ///< Maximum size of the SHADOW buffer to store the received Shadow message
#define MAX_SIZE_OF_UNIQUE_CLIENT_ID_BYTES 80  ///< Maximum size of the Unique Client Id. For More info on the Client Id refer \ref response "Acknowledgments"
#define MAX_SIZE_CLIENT_ID_WITH_SEQUENCE MAX_SIZE_OF_UNIQUE_CLIENT_ID_BYTES + 10 ///< This is size of the extra sequence number that will be appended to the Unique client Id
#define MAX_SIZE_CLIENT_TOKEN_CLIENT_SEQUENCE MAX_SIZE_CLIENT_ID_WITH_SEQUENCE + 20 ///< This is size of the the total clientToken key and value pair in the JSON
#define MAX_ACKS_TO_COMEIN_AT_ANY_GIVEN_TIME 10 ///< At Any given time we will wait for this many responses. This will correlate to the rate at which the shadow actions are requested
#define MAX_THINGNAME_HANDLED_AT_ANY_GIVEN_TIME 10 ///< We could perform shadow action on any thing Name and this is maximum Thing Names we can act on at any given time
#define MAX_JSON_TOKEN_EXPECTED 120 ///< These are the max tokens that is expected to be in the Shadow JSON document. Include the metadata that gets published
#define MAX_SHADOW_TOPIC_LENGTH_WITHOUT_THINGNAME 60 ///< All shadow actions have to be published or subscribed to a topic which is of the format $aws/things/{thingName}/shadow/update/accepted. This refers to the size of the topic without the Thing Name
#define MAX_SIZE_OF_THING_NAME 20 ///< The Thing Name should not be bigger than this value. Modify this if the Thing Name needs to be bigger
#define MAX_SHADOW_TOPIC_LENGTH_BYTES MAX_SHADOW_TOPIC_LENGTH_WITHOUT_THINGNAME + MAX_SIZE_OF_THING_NAME ///< This size includes the length of topic with Thing Name

// Job specific configs
#ifndef DISABLE_IOT_JOBS
#define MAX_SIZE_OF_JOB_ID 64
#define MAX_JOB_JSON_TOKEN_EXPECTED 120
#define MAX_SIZE_OF_JOB_REQUEST AWS_IOT_MQTT_TX_BUF_LEN

#define MAX_JOB_TOPIC_LENGTH_WITHOUT_JOB_ID_OR_THING_NAME 40
#define MAX_JOB_TOPIC_LENGTH_BYTES MAX_JOB_TOPIC_LENGTH_WITHOUT_JOB_ID_OR_THING_NAME + MAX_SIZE_OF_THING_NAME + MAX_SIZE_OF_JOB_ID + 2
#endif
// Auto Reconnect specific config
#define AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL 1000 ///< Minimum time before the First reconnect attempt is made as part of the exponential back-off algorithm
#define AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL 128000 ///< Maximum time interval after which exponential back-off will stop attempting to reconnect.

#if defined(STM32L475xx)
#define SDK_METRICS_PLATFORM_NAME "B-L475E-IOT01A"
#elif defined(STM32L496xx)
#define SDK_METRICS_PLATFORM_NAME "P-L496G-CELL02"
#elif defined(STM32F413xx)
#define SDK_METRICS_PLATFORM_NAME "32F413HDISCOVERY"
#elif defined(STM32F769xx)
#define SDK_METRICS_PLATFORM_NAME "32F769IDISCOVERY"
#else
#define SDK_METRICS_PLATFORM_NAME "STM32Cube"
#endif
#define DISABLE_METRICS false ///< Disable the collection of metrics by setting this to true

// Strings to facilitate creating MQTT Topic names for Thing Shadows
#define AWS_DEVICE_SHADOW_PRE                     "$aws/things/"
#define AWS_DEVICE_SHADOW_UPDATE_TOPIC            "/shadow/update"
#define AWS_DEVICE_SHADOW_UPDATE_ACCEPTED_TOPIC   "/shadow/update/accepted"
#define AWS_DEVICE_SHADOW_UPDATE_DOCUMENTS_TOPIC  "/shadow/update/documents"
#define AWS_DEVICE_SHADOW_UPDATE_REJECTED_TOPIC   "/shadow/update/rejected"
#define AWS_DEVICE_SHADOW_UPDATE_DELTA_TOPIC      "/shadow/update/delta"
#define AWS_DEVICE_SHADOW_GET_TOPIC               "/shadow/get"
#define AWS_DEVICE_SHADOW_GET_ACCEPTED_TOPIC      "/shadow/get/accepted"
#define AWS_DEVICE_SHADOW_GET_REJECTED_TOPIC      "/shadow/get/rejected"
#define AWS_DEVICE_SHADOW_DELETE_TOPIC            "/shadow/delete"
#define AWS_DEVICE_SHADOW_DELETE_ACCEPTED_TOPIC   "/shadow/delete/accepted"
#define AWS_DEVICE_SHADOW_DELETE_REJECTED_TOPIC   "/shadow/delete/rejected"

#endif /* SRC_SHADOW_IOT_SHADOW_CONFIG_H_ */
