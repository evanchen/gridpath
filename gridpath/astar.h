#pragma once

/*
A*算法原理解释:
https://www.gamedev.net/articles/programming/artificial-intelligence/a-pathfinding-for-beginners-r2003/
    1. Drop it from the open list and add it to the closed list.
    2. Check all of the adjacent squares. Ignoring those that are on the closed list or unwalkable (terrain with walls, water, or other illegal terrain), 
        add squares to the open list if they are not on the open list already. Make the selected square the "parent" of the new squares.
    3. If an adjacent square is already on the open list, check to see if this path to that square is a better one. In other words, 
        check to see if the G score for that square is lower if we use the current square to get there. If not, don't do anything.
        On the other hand, if the G cost of the new path is lower, change the parent of the adjacent square to the selected square (in the diagram above, 
        change the direction of the pointer to point at the selected square). Finally, recalculate both the F and G scores of that square. If this seems confusing, 
        you will see it illustrated below.
    4. Add the starting square (or node) to the open list.
    5. Repeat the following:
        a) Look for the lowest F cost square on the open list. We refer to this as the current square.
        b) Switch it to the closed list.
        c) For each of the 8 squares adjacent to this current square ...
            If it is not walkable or if it is on the closed list, ignore it. Otherwise do the following.
            If it isn't on the open list, add it to the open list. Make the current square the parent of this square. Record the F, G, and H costs of the square.
            If it is on the open list already, check to see if this path to that square is better, using G cost as the measure. A lower G cost means that this is a better 
                path. If so, change the parent of the square to the current square, and recalculate the G and F scores of the square. 
                If you are keeping your open list sorted by F score, you may need to resort the list to account for the change.
    
    6. Add the target square to the closed list, in which case the path has been found (see note below), or
    7. Fail to find the target square, and the open list is empty. In this case, there is no path.
    8. Save the path. Working backwards from the target square, go from each square to its parent square until you reach the starting square. 
        That is your path. Note: In earlier versions of this article, it was suggested that you can stop when the target square (or node) has been added
        to the open list, rather than the closed list. Doing this will be faster and it will almost always give you the shortest path, but not always. 
        Situations where doing this could make a difference are when the movement cost to move from the second to the last node to the last (target) node 
        can vary significantly -- as in the case of a river crossing between two nodes, for example.
*/

#include "gridmap.h"
#include<vector>
#include<queue>
#include<cassert>


class astar_search
{
public:
    astar_search(gridmap* gmap,int search_depth) : m_map(gmap), m_search_depth(search_depth) {}
    ~astar_search() {
        if (m_map) {
            delete m_map;
        }
        m_map = nullptr;
    }
public:
    gridmap* get_map() {
        return m_map;
    }

    void find_path(grid* start,grid* target) {
        m_path.clear();
        m_closedlist.clear();
        while (!m_openlist.empty()) m_openlist.pop();

        m_path[start->m_id] = -1; //起点标识
        start->clean_weight(); //起点的g是0
        m_openlist.emplace(*start);
        m_inopenlist[start->m_id] = *start;

        int depth = 0;

        while (!m_openlist.empty()) {
            //1.Drop it from the open list and add it to the closed list.
            grid curgrid = m_openlist.top();
            m_openlist.pop();
            auto it = m_inopenlist.find(curgrid.m_id);
            curgrid = it->second; //这才是真正的grid数值
            m_inopenlist.erase(curgrid.m_id);
            m_closedlist[curgrid.m_id] = true;

            //printf("enclosed: %d\n",curgrid.m_id);

            if (curgrid == *target) {
                break;
            }
            
            depth++;
            if (depth >= m_search_depth) {
                printf("[find_path]: beyond max search depth\n");
                m_path.clear();
                break;
            }
            //2. Check all of the adjacent squares.
            grid* neighbors = m_map->get_neighbors(&curgrid);
            for (int direction = Direction::DIRC_LEFT; direction < Direction::DIRC_MAX; direction++) {
                grid new_nb = neighbors[direction];
                if (new_nb.m_id == -1) {
                    continue;
                }
                if (m_closedlist.find(new_nb.m_id) != m_closedlist.end()) {
                    continue;
                }
                //3. If an adjacent square is already on the open list, check to see if this path to that square is a better one.
                // 
                //如果邻居结点不在openlist, 把它加入到openlist, 记录它的parent为当前结点;
                //如果邻居结点在openlist, 判断是否这个经过 curgrid 的这个邻居的 g,比在openlist里的副本的 g 更小
                auto old_nb = m_inopenlist.find(new_nb.m_id);
                if (old_nb == m_inopenlist.end()) {
                    m_path[new_nb.m_id] = curgrid.m_id;
                    m_map->calculate_h(&new_nb, target);

                    m_inopenlist[new_nb.m_id] = new_nb;
                    m_openlist.emplace(new_nb); //只需要一个副本
                    //printf("new neighbor: %d<-%d\n",new_nb.m_id,curgrid.m_id);
                }
                else {
                    if (new_nb.m_g < old_nb->second.m_g) {
                        assert(new_nb.m_id == old_nb->second.m_id);
                        //经过curgrid的这个新邻居是一个更优的路径,修改它的parent指向curgrid,并更新f
                        m_path[old_nb->second.m_id] = curgrid.m_id;
                        m_map->calculate_h(&new_nb, target);
                        m_inopenlist[new_nb.m_id] = new_nb;
                        //printf("good neighbor: %d<-%d\n", new_nb.m_id, curgrid.m_id);
                    }
                }
            }
        }

        if (m_path.empty()) {
            printf("[find_path]: can not find the path\n");
            return;
        }
    }

    void print_path(grid* start, grid* target) {
        if (m_path.empty()) {
            printf("[print_path]: can not find the path\n");
            return;
        }

        std::map<int, bool> realpath;
        auto it = m_path.find(target->m_id);
        while (it->second != -1) {//从终点往回找到出发点的路径
            int parent = it->second;
            realpath[it->first] = true;
            m_path.erase(it);
            it = m_path.find(parent);
        }

        int gridnum = m_map->get_grid_num();
        const std::vector<grid> allgrids = m_map->get_all_grids();
        int wgrids = m_map->get_wgrids();
        int count = 0;
        for (int i = 0; i < allgrids.size(); i++) {
            const grid& g = allgrids[i];
            if (g == *start) {
                printf(">");
            }
            else if (g == *target) {
                printf("<");
            }
            else if (realpath.find(i) != realpath.end()) {
                printf(">");
            }
            else if (m_path.find(i) != m_path.end()) {
                printf(".");
            }
            else if (m_map->is_block(i)) {
                printf("|");
            }
            else if (m_inopenlist.find(i) != m_inopenlist.end()) {
                printf("i");
            }
            else if (m_closedlist.find(i) != m_closedlist.end()) {
                printf("c");
            }
            else {
                printf("?");
            }
            printf(" ");
            //printf("%d,%d,%d|",g.m_id,g.m_x,g.m_y);
            count++;
            if (count >= wgrids) {
                printf("\n");
                count = 0;
            }
        }
    }
private:
    gridmap* m_map;
    int m_search_depth;
    std::map<int,bool> m_closedlist;
    std::map<int, grid> m_inopenlist;
    std::priority_queue<grid, std::vector<grid>, cmp> m_openlist;
    std::map<int,int> m_path;
};
