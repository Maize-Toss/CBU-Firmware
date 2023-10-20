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
    int score = 0;
    int state = 0;
}GameInfo;

// Define a struct to hold the deserialized values
typedef struct GameInfo {
    struct TeamInfo team1 = {0,0};
    struct TeamInfo team2 = {0,0};
    int end_of_round = 0;
}GameInfo;

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

#endif /* INC_MAIZEJSON_H_ */
