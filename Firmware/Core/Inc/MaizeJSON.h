/*
 * MaizeJSON.h
 *
 *  Created on: Oct 19, 2023
 *      Author: evanm
 */

#ifndef INC_MAIZEJSON_H_
#define INC_MAIZEJSON_H_

#include <stdlib.h>
#include "cJSON.h"

// Define a struct to hold the deserialized values for each team
typedef struct TeamInfo {
    int score;
    int state;
}TeamInfo;

// Define a struct to hold the deserialized values
typedef struct GameInfo {
    struct TeamInfo red;
    struct TeamInfo blue;
    int end_of_round;
}GameInfo;


typedef struct BroadcastPacket {
	uint32_t batteryVoltage;
	uint32_t redDeltaScore;
	uint32_t blueDeltaScore;
} BroadcastPacket;


/**
 * @brief Deserialize JSON data and fill a struct with game information.
 *
 * This function takes a JSON string and fills a GameInfo struct with the deserialized information.
 *
 * @param json_data The JSON data to be deserialized.
 * @param info Pointer to the GameInfo struct to store the deserialized information.
 */
void deserializeJSON(const char* json_data, struct GameInfo* info) {
    cJSON *root = cJSON_Parse(json_data);

    // Who is JSON?

    if (root == NULL) {
        // Handle the error here, e.g., set default values or return.
        return;
    }

    cJSON *red_json = cJSON_GetObjectItem(root, "team1");
    cJSON *blue_json = cJSON_GetObjectItem(root, "team2");
    cJSON *end_of_round_json = cJSON_GetObjectItem(root, "end_of_round");

    if (red_json && cJSON_IsObject(red_json)) {
        cJSON *score_json = cJSON_GetObjectItem(red_json, "score");
        cJSON *state_json = cJSON_GetObjectItem(red_json, "state");

        if (score_json && cJSON_IsNumber(score_json)) {
            info->red.score = score_json->valueint;
        }

        if (state_json && cJSON_IsNumber(state_json)) {
            info->red.state = state_json->valueint;
        }
    }

    if (blue_json && cJSON_IsObject(blue_json)) {
        cJSON *score_json = cJSON_GetObjectItem(blue_json, "score");
        cJSON *state_json = cJSON_GetObjectItem(blue_json, "state");

        if (score_json && cJSON_IsNumber(score_json)) {
            info->blue.score = score_json->valueint;
        }

        if (state_json && cJSON_IsNumber(state_json)) {
            info->blue.state = state_json->valueint;
        }
    }
    if (end_of_round_json && cJSON_IsBool(end_of_round_json)) {
        info->end_of_round = end_of_round_json->valueint;
    }

    cJSON_Delete(root);
}



/**
 * @brief Serialize JSON data and fill a struct with game information.
 *
 * This function takes a broadcast packet and serializes it into a JSON and stores it in a char array.
 *
 * @param packet Pointer to he JSON data to be seralized.
 * @param data Pointer to the string storing the serialized JSON information.
 */
void serializeJSON(BroadcastPacket* data, char* dst ){

	 cJSON* json = cJSON_CreateObject();

	    if (json == NULL) {
//	        fprintf(stderr, "Failed to create JSON object.\n");
	        return NULL;
	    }

	    cJSON_AddNumberToObject(json, "battery", data->batteryVoltage); // battery voltage
	    cJSON_AddNumberToObject(json, "team1d", data->redDeltaScore);
	    cJSON_AddNumberToObject(json, "team2d", data->blueDeltaScore);

	    char* temp = cJSON_PrintUnformatted(json);
	    strncat (temp, "\n", 2);
	    strcpy(dst, temp);
	    cJSON_Delete(json);

	    return dst;


}

#endif /* INC_MAIZEJSON_H_ */
