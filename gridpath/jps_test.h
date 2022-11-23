#pragma once
#include "gridmap.h"
#include "jps.h"

void jps_test() {
    printf("======= jps_test begin =======\n");
    int width = 923;
    int height = 943;
    gridmap* gmap = new gridmap(width, height);
    gmap->init();
    int blocks[6][4] = {
        {65,75,100,380},
        {165,275,231,880},
        {465,35,600,480},
        {475,530,600,580},
        {145,345,211,395},
        {295,345,751,395},
    };
    for (int i = 0; i < 6; i++) {
        int* pos = blocks[i];
        gmap->build_bocks(pos[0], pos[1], pos[2], pos[3]);
    }

    int search_depth = 10000;
    jps_search* jps = new jps_search(gmap, search_depth);

    int startx = 0;
    int starty = 0;
    int targetx = 793;
    int targety = 880;

    int starti = gmap->get_grid_index_by_distance_pos(startx, starty);
    int targeti = gmap->get_grid_index_by_distance_pos(targetx, targety);
    grid* start = gmap->get_grid(starti);
    grid* target = gmap->get_grid(targeti);
    jps->find_path(start, target);
    jps->print_path(start, target);
    printf("======= jps_test end =======\n");
}
