#pragma once
/*

jps �㷨ԭ��:
https://zerowidth.com/2013/a-visual-explanation-of-jump-point-search.html

����A*�Ĳ�ͬ,���ڼ��ٶԼ���openlist�Ľ�������.
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
        //̽�⵱ǰ���ӵ��Ϸ����·��Ƿ������赲
        int x = start.m_x;
        int y = start.m_y + vstep; //��������(0,0),x������,y������
        int index = m_map->get_grid_index_by_grid_pos(x, y);
        if (m_map->is_block(index)) {
            //�Ƿ���ǿ���ھ�
            x = x + hstep;
            index = m_map->get_grid_index_by_grid_pos(x, y);
            if (!m_map->is_block(index)) {
                //����ǿ���ھ�(�ɿ��ǵ�·����)
                return true;
            }
        }
        return false;
    }

    //ˮƽ����̽��
    int probe_horizontal(const grid& target,const grid& parent, grid& start, int hstep) {
        //��������п��ܽ���openlist��,�������¼�������parent��g
        start.m_g = std::abs(parent.m_x - start.m_x) * STRAIGHT_COST;
        m_map->calculate_g(&start, &parent);

        if (start == target) {
            put2openlist(target, parent, start);
            return start.m_id;
        }

        if (is_probed(start.m_id)) return -1;
        mark_probed(start.m_id);

        //̽��ǰ�������Ƿ����赲
        int fx = start.m_x + hstep;
        int fy = start.m_y;
        int findex = m_map->get_grid_index_by_grid_pos(fx, fy);
        if (m_map->is_block(findex)) {
            return -1;
        }
        
        //�Ϸ�
        if (is_horizontal_forceneighbor(start, hstep, -1)) {
            put2openlist(target, parent, start);
            return start.m_id;
        }
        //�·�
        if (is_horizontal_forceneighbor(start, hstep, 1)) {
            put2openlist(target, parent, start);
            return start.m_id;
        }

        //������ǰһ��
        return findex;
    }

    bool is_vertical_forceneighbor(const grid& start, int hstep, int vstep) {
        //̽�⵱ǰ���ӵ��󷽻��ҷ��Ƿ������赲
        int x = start.m_x + hstep;
        int y = start.m_y; //��������(0,0),x������,y������
        int index = m_map->get_grid_index_by_grid_pos(x, y);
        if (m_map->is_block(index)) {
            //�Ƿ���ǿ���ھ�
            y = y + vstep;
            index = m_map->get_grid_index_by_grid_pos(x, y);
            if (!m_map->is_block(index)) {
                //����ǿ���ھ�(�ɿ��ǵ�·����)
                return true;
            }
        }
        return false;
    }

    //��ֱ����̽��
    int probe_vertical(const grid& target, const grid& parent, grid& start, int vstep) {
        //��������п��ܽ���openlist��,�������¼�������parent��g
        start.m_g = std::abs(parent.m_x - start.m_x) * STRAIGHT_COST;
        m_map->calculate_g(&start, &parent);
        
        if (start == target) {
            put2openlist(target, parent, start);
            return start.m_id;
        }

        if (is_probed(start.m_id)) return -1;
        mark_probed(start.m_id);

        //̽��ǰ�������Ƿ����赲
        int fx = start.m_x;
        int fy = start.m_y + vstep;
        int findex = m_map->get_grid_index_by_grid_pos(fx, fy);
        if (m_map->is_block(findex)) {
            return -1;
        }

        //��
        if (is_vertical_forceneighbor(start, -1, vstep)) {
            put2openlist(target, parent, start);
            return start.m_id;
        }
        //�ҷ�
        if (is_vertical_forceneighbor(start, 1, vstep)) {
            put2openlist(target, parent, start);
            return start.m_id;
        }

        //������ǰһ��
        return findex;
    }

    //б�߷���̽��
    int probe_diagonal(const grid& target, const grid& origin_parent, grid& start, int hstep, int vstep) {
        //start �� parent �ܱ��ھӵ�һ��б�ߵ�,����û�б�̽�����.
        //����ֻ��Ҫ��б����ֽ�Ϊˮƽ/��ֱ����,������������Ҫ�� parent ��ˮƽ/��ֱ������ھ�һ��̽����ȥ.
        //������Ҫб�߷��������赲Ϊֹ�ſ���ͣ����.

        //��������п��ܽ���openlist��,�������¼�������parent��g
        start.m_g = std::abs(origin_parent.m_x - start.m_x) * DIAGONAL_COST;
        m_map->calculate_g(&start,&origin_parent);
        
        if (start == target) {
            put2openlist(target, origin_parent, start);
            return start.m_id;
        }

        if (is_probed(start.m_id)) return -1;
        mark_probed(start.m_id);

        //�� start Ϊ���,�ֱ�̽��ֽⷽ��
        //ˮƽǰ���������һ����
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
                    //�����������openlist,����parentҲҪ���뵽openlist
                    put2openlist(target, origin_parent, start);
                    break;
                }
                green = *(m_map->get_grid(forward_index));
            }
        }
        
        //��ֱǰ���������һ����
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
                    //�����������openlist,����parentҲҪ���뵽openlist
                    put2openlist(target, origin_parent, start);
                    break;
                }
                green = *(m_map->get_grid(forward_index));
            }
        }

        //б�߷���ǰ��һ��
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

        m_path[start->m_id] = -1; //����ʶ
        start->clean_weight(); //����g��0
        m_openlist.emplace(*start);
        m_inopenlist[start->m_id] = *start;

        int depth = 0;
        while (!m_openlist.empty()) {
            grid curgrid = m_openlist.top();
            m_openlist.pop();
            auto it = m_inopenlist.find(curgrid.m_id);
            curgrid = it->second; //�����������grid��ֵ(f,g,h)
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
                        new_nb = *(m_map->get_grid(forward_index)); //б�߷���ǰ��һ����Ϊ�µ�б��������ʼ��
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
            m_openlist.emplace(new_nb); //ֻ��Ҫһ������
            //printf("new neighbor: %d<-%d\n",new_nb.m_id,curgrid.m_id);
        }
        else {
            if (new_nb.m_g < old_nb->second.m_g) {
                assert(new_nb.m_id == old_nb->second.m_id);
                //����curgrid��������ھ���һ�����ŵ�·��,�޸�����parentָ��curgrid,������f
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
        while (it->second != -1) {//���յ������ҵ��������·��
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

