#ifndef READMAP_H
#define READMAP_H

#include <string>

/**
 * @brief Reads a 2D map from a file and stores it in a dynamically allocated array.
 *
 * @param filepath The path to the map file.
 * @param stateMap Pointer to the 2D array to store the map.
 * @param row_size Pointer to store the number of rows in the map.
 * @param col_size Pointer to store the number of columns in the map.
 * @return True if the map was successfully read, false otherwise.
 */
bool readMap(const std::string &filepath, int ***stateMap, int *row_size, int *col_size);

/**
 * @brief Reads a 3D map from a file and stores it in a dynamically allocated array.
 *
 * @param filepath The path to the 3D map file.
 * @param stateMap Pointer to the 3D array to store the map.
 * @param row_size Pointer to store the number of rows in the map.
 * @param col_size Pointer to store the number of columns in the map.
 * @param height_size Pointer to store the height of the map.
 * @return True if the 3D map was successfully read, false otherwise.
 */
bool read3DMap(const std::string &filepath, int ****stateMap, int *row_size, int *col_size, int *height_size);

/**
 * @brief Frees the memory allocated for a 2D map.
 *
 * @param map The 2D array to free.
 * @param row_size The number of rows in the map.
 */
void freeMap(int **map, int row_size);

/**
 * @brief Frees the memory allocated for a 3D map.
 *
 * @param map The 3D array to free.
 * @param row_size The number of rows in the map.
 * @param col_size The number of columns in the map.
 */
void free3DMap(int ***map, int row_size, int col_size);

#endif // READMAP_H