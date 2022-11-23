#pragma once
/*

jps 算法原理:
https://zerowidth.com/2013/a-visual-explanation-of-jump-point-search.html

它与A*的不同,在于减少对加入openlist的结点的数量.
*/


#include "gridmap.h"
#include<vector>
#include<queue>
#include<cassert>

extern const int STRAIGHT_COST;
extern const int DIAGONAL_COST;

class jps_search
{
public:
    jps_search(gridmap* gmap, int search_depth) : m_map(gmap), m_search_depth(search_depth) {}
    ~jps_search() {
        if (m_map) {
            delete m_map;
        }
        m_map = nullptr;
    }
public:
    gridmap* get_map() {
        return m_map;
    }

    bool is_probed(int index) {
        return m_probed.find(index) != m_probed.end();
    }

    void mark_probed(int index) {
        m_probed[index] = true;
    }

    bool is_horizontal_forceneighbor(const grid& start, int hstep, int vstep) {
        //探测当前格子的上方或下方是否是有阻挡
        int x = start.m_x;
        int y = start.m_y + vstep; //坐标轴是(0,0),x轴向右,y轴向下
        int index = m_map->get_grid_index_by_grid_pos(x, y);
        if (m_map->is_block(index)) {
            //是否有强迫邻居
            x = x + hstep;
            index = m_map->get_grid_index_by_grid_pos(x, y);
            if (!m_map->is_block(index)) {
                //存在强迫邻居(可考虑的路径点)
                return true;
            }
        }
        return false;
    }

    //水平方向探测
    int probe_horizontal(const grid& target,const grid& parent, grid& start, int hstep) {
        //这个点是有可能进入openlist的,所以重新计算它与parent的g
        start.m_g = std::abs(parent.m_x - start.m_x) * STRAIGHT_COST;
        m_map->calculate_g(&start, &parent);

        if (start == target) {
            put2openlist(target, parent, start);
            return start.m_id;
        }

        if (is_probed(start.m_id)) return -1;
        mark_probed(start.m_id);

        //探测前方格子是否是阻挡
        int fx = start.m_x + hstep;
        int fy = start.m_y;
        int findex = m_map->get_grid_index_by_grid_pos(fx, fy);
        if (m_map->is_block(findex)) {
            return -1;
        }
        
        //上方
        if (is_horizontal_forceneighbor(start, hstep, -1)) {
            put2openlist(target, parent, start);
            return start.m_id;
        }
        //下方
        if (is_horizontal_forceneighbor(start, hstep, 1)) {
            put2openlist(target, parent, start);
            return start.m_id;
        }

        //返回向前一格
        return findex;
    }

    bool is_vertical_forceneighbor(const grid& start, int hstep, int vstep) {
        //探测当前格子的左方或右方是否是有阻挡
        int x = start.m_x + hstep;
        int y = start.m_y; //坐标轴是(0,0),x轴向右,y轴向下
        int index = m_map->get_grid_index_by_grid_pos(x, y);
        if (m_map->is_block(index)) {
            //是否有强迫邻居
            y = y + vstep;
            index = m_map->get_grid_index_by_grid_pos(x, y);
            if (!m_map->is_block(index)) {
                //存在强迫邻居(可考虑的路径点)
                return true;
            }
        }
        return false;
    }

    //竖直方向探测
    int probe_vertical(const grid& target, const grid& parent, grid& start, int vstep) {
        //这个点是有可能进入openlist的,所以重新计算它与parent的g
        start.m_g = std::abs(parent.m_x - start.m_x) * STRAIGHT_COST;
        m_map->calculate_g(&start, &parent);
        
        if (start == target) {
            put2openlist(target, parent, start);
            return start.m_id;
        }

        if (is_probed(start.m_id)) return -1;
        mark_probed(start.m_id);

        //探测前方格子是否是阻挡
        int fx = start.m_x;
        int fy = start.m_y + vstep;
        int findex = m_map->get_grid_index_by_grid_pos(fx, fy);
        if (m_map->is_block(findex)) {
            return -1;
        }

        //左方
        if (is_vertical_forceneighbor(start, -1, vstep)) {
            put2openlist(target, parent, start);
            return start.m_id;
        }
        //右方
        if (is_vertical_forceneighbor(start, 1, vstep)) {
            put2openlist(target, parent, start);
            return start.m_id;
        }

        //返回向前一格
        return findex;
    }

    //斜线方向探测
    int probe_diagonal(const grid& target, const grid& origin_parent, grid& start, int hstep, int vstep) {
        //start 是 parent 周边邻居的一个斜线点,而且没有被探测过的.
        //我们只需要把斜向方向分解为水平/竖直方向,这两个方向需要跟 parent 的水平/竖直方向的邻居一样探测下去.
        //但是需要斜线方向遇到阻挡为止才可以停下来.

        //这个点是有可能进入openlist的,所以重新计算它与parent的g
        start.m_g = std::abs(origin_parent.m_x - start.m_x) * DIAGONAL_COST;
        m_map->calculate_g(&start,&origin_parent);
        
        if (start == target) {
            put2openlist(target, origin_parent, start);
            return start.m_id;
        }

        if (is_probed(start.m_id)) return -1;
        mark_probed(start.m_id);

        //以 start 为起点,分别探测分解方向
        //水平前进方向的下一个点
        int hx = start.m_x + hstep;
        int hy = start.m_y;
        int hindex = m_map->get_grid_index_by_grid_pos(hx, hy);
        if (!m_map->is_block(hindex)) {
            int forward_index = -1;
            grid green = *(m_map->get_grid(hindex));
            while (true) {
                forward_index = probe_horizontal(target, start, green, hstep);
                if (forward_index == -1) break;
                if (forward_index == green.m_id) {
                    //这个结点加入了openlist,它的parent也要加入到openlist
                    put2openlist(target, origin_parent, start);
                    break;
                }
                green = *(m_map->get_grid(forward_index));
            }
        }
        
        //竖直前进方向的下一个点
        int vx = start.m_x;
        int vy = start.m_y + vstep;
        int vindex = m_map->get_grid_index_by_grid_pos(vx, vy);
        if (!m_map->is_block(vindex)) {
            int forward_index = -1;
            grid green = *(m_map->get_grid(vindex));
            while (true) {
                forward_index = probe_vertical(target, start, green, vstep);
                if (forward_index == -1) break;
                if (forward_index == green.m_id) {
                    //这个结点加入了openlist,它的parent也要加入到openlist
                    put2openlist(target, origin_parent, start);
                    break;
                }
                green = *(m_map->get_grid(forward_index));
            }
        }

        //斜线方向前进一格
        int x = start.m_x + hstep;
        int y = start.m_y + vstep;
        int index = m_map->get_grid_index_by_grid_pos(x, y);
        if (m_map->is_block(index)) {
            return -1;
        }
        return index;
    }

    void find_path(grid* start, grid* target) {
        m_path.clear();
        m_closedlist.clear();
        while (!m_openlist.empty()) m_openlist.pop();

        m_path[start->m_id] = -1; //起点标识
        start->clean_weight(); //起点的g是0
        m_openlist.emplace(*start);
        m_inopenlist[start->m_id] = *start;

        int depth = 0;
        while (!m_openlist.empty()) {
            grid curgrid = m_openlist.top();
            m_openlist.pop();
            auto it = m_inopenlist.find(curgrid.m_id);
            curgrid = it->second; //这才是真正的grid数值(f,g,h)
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

            grid* neighbors = m_map->get_neighbors(&curgrid);
            for (int direction = Direction::DIRC_LEFT; direction < Direction::DIRC_MAX; direction++) {
                grid new_nb = neighbors[direction];
                if (new_nb.m_id == -1) {
                    continue;
                }
                if (m_closedlist.find(new_nb.m_id) != m_closedlist.end()) {
                    continue;
                }

                const int* dir = get_dir(direction);
                int hstep = dir[0];
                int vstep = dir[1];
                
                const grid& origin_parent = curgrid;
                int forward_index = -1;
                switch (direction) {
                case Direction::DIRC_LEFT:
                case Direction::DIRC_RIGHT:
                    while (true) {
                        forward_index = probe_horizontal(*target, origin_parent, new_nb, hstep);
                        if (forward_index == -1 || forward_index == new_nb.m_id) break;
                        new_nb = *(m_map->get_grid(forward_index));
                    }
                    break;
                case Direction::DIRC_UP:
                case Direction::DIRC_DOWN:
                    while (true) {
                        forward_index = probe_vertical(*target, origin_parent, new_nb, vstep);
                        if (forward_index == -1 || forward_index == new_nb.m_id) break;
                        new_nb = *(m_map->get_grid(forward_index));
                    }
                    break;
                case Direction::DIRC_LEFT_UP:
                case Direction::DIRC_LEFT_DOWN:
                case Direction::DIRC_RIGHT_UP:
                case Direction::DIRC_RIGHT_DOWN:
                    while (true) {
                        forward_index = probe_diagonal(*target, origin_parent, new_nb, hstep, vstep);
                        if (forward_index == -1 || forward_index == new_nb.m_id) break;
                        new_nb = *(m_map->get_grid(forward_index)); //斜线方向前进一格作为新的斜线搜索开始点
                    }
                    break;
                default:
                    break;
                }
            }
        }

        if (m_path.empty()) {
            printf("[find_path]: can not find the path\n");
            return;
        }
    }

    void put2openlist(const grid& target,const grid parent,grid& new_nb) {
        auto old_nb = m_inopenlist.find(new_nb.m_id);
        if (old_nb == m_inopenlist.end()) {
            m_path[new_nb.m_id] = parent.m_id;
            m_map->calculate_h(&new_nb, &target);

            m_inopenlist[new_nb.m_id] = new_nb;
            m_openlist.emplace(new_nb); //只需要一个副本
            //printf("new neighbor: %d<-%d\n",new_nb.m_id,curgrid.m_id);
        }
        else {
            if (new_nb.m_g < old_nb->second.m_g) {
                assert(new_nb.m_id == old_nb->second.m_id);
                //经过curgrid的这个新邻居是一个更优的路径,修改它的parent指向curgrid,并更新f
                m_path[old_nb->second.m_id] = parent.m_id;
                m_map->calculate_h(&new_nb, &target);
                m_inopenlist[new_nb.m_id] = new_nb;
                //printf("good neighbor: %d<-%d\n", new_nb.m_id, curgrid.m_id);
            }
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
                printf("o");
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
            else if (m_probed.find(i) != m_probed.end()) {
                printf(".");
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
    std::map<int, bool> m_closedlist;
    std::map<int, grid> m_inopenlist;
    std::priority_queue<grid, std::vector<grid>, cmp> m_openlist;
    std::map<int, int> m_path;
    std::map<int, bool> m_probed;
};

