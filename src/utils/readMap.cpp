#include "readMap.h"
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <boost/algorithm/string.hpp>

/**
 * @brief Reads a 2D map from a file and stores it in a dynamically allocated array.
 */
bool readMap(const std::string &filepath, int ***stateMap, int *row_size, int *col_size)
{
    std::ifstream fin(filepath);
    if (!fin.is_open())
    {
        return false;
    }

    std::string line;

    // Read row size
    getline(fin, line);
    if (line.substr(0, line.find(" ")) != "height")
    {
        std::cerr << "Incorrect format: Expected height, got " << line.substr(0, line.find(" ")) << std::endl;
        return false;
    }
    *row_size = std::stoi(line.substr(line.find(" ") + 1));

    // Read column size
    getline(fin, line);
    if (line.substr(0, line.find(" ")) != "width")
    {
        std::cerr << "Incorrect format: Expected width, got " << line.substr(0, line.find(" ")) << std::endl;
        return false;
    }
    *col_size = std::stoi(line.substr(line.find(" ") + 1));

    // Read free and obstacle characters
    getline(fin, line);
    std::string free_char = line.substr(line.find(" ") + 1);

    getline(fin, line);
    std::string obs_char = line.substr(line.find(" ") + 1);

    // Allocate memory for the map
    int **map = new int *[*row_size];
    for (int i = 0; i < *row_size; ++i)
    {
        map[i] = new int[*col_size];
    }

    // Read the map data
    int curr_row = 0;
    while (getline(fin, line))
    {
        for (int curr_col = 0; curr_col < *col_size; ++curr_col)
        {
            char c = line[curr_col];
            if (c == free_char[0])
            {
                map[curr_row][curr_col] = 0; // Free cell
            }
            else if (c == obs_char[0])
            {
                map[curr_row][curr_col] = 1; // Obstacle
            }
        }
        ++curr_row;
    }

    *stateMap = map;
    fin.close();
    return true;
}

/**
 * @brief Reads a 3D map from a file and stores it in a dynamically allocated array.
 */
bool read3DMap(const std::string &filepath, int ****stateMap, int *row_size, int *col_size, int *height_size)
{
    std::ifstream fin(filepath);
    if (!fin.is_open())
    {
        return false;
    }

    std::string line;
    std::vector<std::string> tokens;

    // Read dimensions
    getline(fin, line);
    boost::split(tokens, line, boost::is_any_of(" "));
    if (tokens[0] != "voxel")
    {
        std::cerr << "Incorrect format: Expected voxel, got " << tokens[0] << std::endl;
        return false;
    }
    *row_size = std::stoi(tokens[1]);
    *col_size = std::stoi(tokens[2]);
    *height_size = std::stoi(tokens[3]);

    // Allocate memory for the 3D map
    int ***map = new int **[*row_size];
    for (int i = 0; i < *row_size; ++i)
    {
        map[i] = new int *[*col_size];
        for (int j = 0; j < *col_size; ++j)
        {
            map[i][j] = new int[*height_size]();
        }
    }

    // Read the map data
    while (getline(fin, line))
    {
        boost::split(tokens, line, boost::is_any_of(" "));
        int curr_row = std::stoi(tokens[0]);
        int curr_col = std::stoi(tokens[1]);
        int curr_height = std::stoi(tokens[2]);
        map[curr_row][curr_col][curr_height] = 1; // Mark as occupied
    }

    *stateMap = map;
    fin.close();
    return true;
}

/**
 * @brief Frees the memory allocated for a 2D map.
 */
void freeMap(int **map, int row_size)
{
    for (int i = 0; i < row_size; ++i)
    {
        delete[] map[i];
    }
    delete[] map;
}

/**
 * @brief Frees the memory allocated for a 3D map.
 */
void free3DMap(int ***map, int row_size, int col_size)
{
    for (int i = 0; i < row_size; ++i)
    {
        for (int j = 0; j < col_size; ++j)
        {
            delete[] map[i][j];
        }
        delete[] map[i];
    }
    delete[] map;
}