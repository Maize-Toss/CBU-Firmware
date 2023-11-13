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
    struct TeamInfo team1;
    struct TeamInfo team2;
    int end_of_round;
}GameInfo;


typedef struct BroadcastPacket {
	uint32_t batteryVoltage;
	uint32_t team0DeltaScore;
	uint32_t team1DeltaScore;
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

    cJSON *team1_json = cJSON_GetObjectItem(root, "team1");
    cJSON *team2_json = cJSON_GetObjectItem(root, "team2");
    cJSON *end_of_round_json = cJSON_GetObjectItem(root, "end_of_round");

    if (team1_json && cJSON_IsObject(team1_json)) {
        cJSON *score_json = cJSON_GetObjectItem(team1_json, "score");
        cJSON *state_json = cJSON_GetObjectItem(team1_json, "state");

        if (score_json && cJSON_IsNumber(score_json)) {
            info->team1.score = score_json->valueint;
        }

        if (state_json && cJSON_IsNumber(state_json)) {
            info->team1.state = state_json->valueint;
        }
    }

    if (team2_json && cJSON_IsObject(team2_json)) {
        cJSON *score_json = cJSON_GetObjectItem(team2_json, "score");
        cJSON *state_json = cJSON_GetObjectItem(team2_json, "state");

        if (score_json && cJSON_IsNumber(score_json)) {
            info->team2.score = score_json->valueint;
        }

        if (state_json && cJSON_IsNumber(state_json)) {
            info->team2.state = state_json->valueint;
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
	    cJSON_AddNumberToObject(json, "team1d", data->team0DeltaScore); // Team0 score delta
	    cJSON_AddNumberToObject(json, "team2d", data->team1DeltaScore);

	    dst = cJSON_Print(json);
	    cJSON_Delete(json);

	    return dst;


}

#endif /* INC_MAIZEJSON_H_ */
