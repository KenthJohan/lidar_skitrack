#include <unistd.h>
#include <stdio.h>
#include <inttypes.h>
#include <mosquitto.h>

#include "csc/csc_crossos.h"
#include "csc/csc_argv.h"

#include "skitrack.h"

#define ARG_HELP             UINT32_C(0x00000001)
#define ARG_VERBOSE          UINT32_C(0x00000002)
#define ARG_STDIN            UINT32_C(0x00000010)

static void on_connect (struct mosquitto *mosq, void *obj, int reason_code)
{
	UNUSED (obj);
	printf("on_connect: %s\n", mosquitto_connack_string (reason_code));
	if (reason_code != 0)
	{
		mosquitto_disconnect (mosq);
	}
}

static void on_publish (struct mosquitto *mosq, void *obj, int mid)
{
	UNUSED (mosq);
	UNUSED (obj);
	//printf ("Message with mid %d has been published.\n", mid);
}


static void publish_float (struct mosquitto *mosq, int qos, char const * topic, float number)
{
	char payload[20];
	snprintf (payload, sizeof(payload), "%f", number);
	printf ("Msg %s: %s\n", topic, payload);
	int rc = mosquitto_publish (mosq, NULL, topic, strlen (payload), payload, qos, 0);
	if (rc != MOSQ_ERR_SUCCESS)
	{
		fprintf (stderr, "Error publishing: %s\n", mosquitto_strerror (rc));
	}
}


int main (int argc, char const * argv[])
{
	UNUSED (argc);
	csc_crossos_enable_ansi_color();

	{
		int r = mosquitto_lib_init();
		ASSERT (r == MOSQ_ERR_SUCCESS);
	}

	uint32_t arg_flags = 0;
	char const * arg_filename = NULL;
	char const * arg_mqtt_address = "test.mosquitto.org";
	uint32_t arg_mqtt_port = 1883;
	uint32_t arg_mqtt_keepalive = 60;
	uint32_t arg_mqtt_qos = 2;
	uint32_t arg_udelay = 0;
	FILE * lidarfile = NULL;
	struct csc_argv_option option[] =
	{
	{CSC_ARGV_DEFINE_GROUP("General:")},
	{'h', "help",       CSC_TYPE_U32,    &arg_flags,          ARG_HELP,    "Show help"},
	{'v', "verbose",    CSC_TYPE_U32,    &arg_flags,          ARG_VERBOSE, "Show verbose"},
	{'i', "stdin",      CSC_TYPE_U32,    &arg_flags,          ARG_STDIN,   "Read pointcloud from stdin"},
	{'f', "filename",   CSC_TYPE_STRING, &arg_filename,       0,           "Read pointcloud from a file"},
	{'D', "delay",      CSC_TYPE_U32,    &arg_udelay,         0,           "Delay in microseconds"},
	{CSC_ARGV_DEFINE_GROUP("MQTT:")},
	{'a', "address",    CSC_TYPE_STRING, &arg_mqtt_address,   0,           "MQTT address"},
	{'p', "port",       CSC_TYPE_U32,    &arg_mqtt_port,      0,           "MQTT port"},
	{'k', "keepalive",  CSC_TYPE_U32,    &arg_mqtt_keepalive, 0,           "MQTT keepalive"},
	{'q', "qos",        CSC_TYPE_U32,    &arg_mqtt_qos,       0,           "MQTT integer value 0, 1 or 2 indicating the Quality of Service to be used for the message."},
	{CSC_ARGV_END}};
	csc_argv_parseall (argv+1, option);

	if (arg_flags & ARG_HELP)
	{
		csc_argv_description0 (option, stdout);
		csc_argv_description1 (option, stdout);
		return 0;
	}


	struct mosquitto * mosq = mosquitto_new (NULL, 1, NULL);
	if (mosq == NULL)
	{
		fprintf (stderr, "Error: Out of memory.\n");
		return 1;
	}


	mosquitto_connect_callback_set (mosq, on_connect);
	mosquitto_publish_callback_set (mosq, on_publish);


	if (mosq != NULL)
	{
		int rc = mosquitto_connect (mosq, arg_mqtt_address, arg_mqtt_port, arg_mqtt_keepalive);
		if (rc != MOSQ_ERR_SUCCESS)
		{
			mosquitto_destroy (mosq);
			fprintf (stderr, "Error: %s\n", mosquitto_strerror(rc));
			return 1;
		}
	}

	{
		int rc = mosquitto_loop_start(mosq);
		if (rc != MOSQ_ERR_SUCCESS)
		{
			mosquitto_destroy (mosq);
			fprintf (stderr, "Error: %s\n", mosquitto_strerror (rc));
			return 1;
		}
	}


	if (arg_flags & ARG_STDIN)
	{
		printf ("[INFO] Opening stdin to read LiDAR frames continuously.\n");
		lidarfile = stdin;
	}
	else if (arg_filename)
	{
		printf ("[INFO] Opening file %s to read LiDAR frames continuously.\n", arg_filename);
		lidarfile = fopen (arg_filename, "rb");
		ASSERT_NOTNULL (lidarfile);
	}
	uint32_t framenr = 0;
	uint32_t framemax = 0;
	if (lidarfile)
	{
		if (arg_flags & ARG_VERBOSE)
		{
			fseek (lidarfile, 0, SEEK_END);
			long int nbytes = ftell (lidarfile);
			framemax = nbytes / (sizeof (float) * LIDAR_WH * POINT_STRIDE);
			fseek (lidarfile, 0, SEEK_SET);
		}
		struct skitrack ski = {0};
		while (1)
		{
			if (arg_udelay > 0)
			{
				usleep (arg_udelay);
			}
			if (arg_flags & ARG_VERBOSE)
			{
				printf ("Framenr %i of %i\n", framenr, framemax);
			}
			int r = fread (ski.pc1, sizeof (float) * LIDAR_WH * POINT_STRIDE, 1, lidarfile);
			ASSERTF (r == 1, "fread %i", r);
			framenr ++;
			ski.pc_count = LIDAR_WH;
			skitrack_process (&ski);

			printf ("[INFO] PC Count: %i of %i\n", ski.pc_count, LIDAR_WH);
			printf ("[INFO] Trackpos: %f\n", ski.trackpos[0]);
			printf ("[INFO] Confidence: %3.0f%%\n", ski.confidence*100.0f);
			printf ("[INFO] Strength: [%i] %f, Threshold=%f\n", ski.peak_u32[0], ski.strength, SKITRACK_STRENGHT_THRESHOLD);

			//Send skitrack info to MQTT
			float speed = 0.0f;
			if ((ski.confidence > 0.5f) && (ski.strength > SKITRACK_STRENGHT_THRESHOLD))
			{
				speed = (ski.confidence - 0.5f) * 2.0f;
			}
			publish_float (mosq, arg_mqtt_qos, "/command/c2h/lidar/offset", ski.trackpos[0]);
			publish_float (mosq, arg_mqtt_qos, "/command/c2h/lidar/angle", atanf (ski.k));
			publish_float (mosq, arg_mqtt_qos, "/command/c2h/lidar/speed", speed);
		}
	}

	return 0;
}
