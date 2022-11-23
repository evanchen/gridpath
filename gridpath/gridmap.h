#pragma once
#include "grid.h"
#include<cstdlib>
#include<vector>
#include<map>
#include<cassert>

// ��ά��ͼ���ӱ߳�(���ٸ����ش�С)
const int GRID_PIXEL = 30;

//�����ھӵ�8������
//ֱ�߷���
enum Direction
{
    DIRC_LEFT				= 0,
    DIRC_RIGHT				= 1,
    DIRC_UP					= 2,
    DIRC_DOWN			=3,
    DIRC_LEFT_UP			= 4,
    DIRC_LEFT_DOWN	= 5,
    DIRC_RIGHT_UP		= 6,
    DIRC_RIGHT_DOWN	= 7,
	DIRC_MAX				= 8,
};


// DIRC_MAX �������Լ������Ӧ�Ŀ���
const int DIR[DIRC_MAX][2] = {
	{-1,0},
	{1,0},
	{0,-1}, //����ע��һ��,Ĭ�ϵ�ǰ����ϵ(0,0)�������Ͻ�,x����������,y����������
	{0,1},
	{-1,-1},
	{ -1,1 },
	{ 1,-1 },
	{ 1,1 }
};

const int* get_dir(int direction) {
	return DIR[direction];
}

//F = G + H, ���Ƕ�Ӧ DIRC_MAX �������ھӵ� G ����
extern const int STRAIGHT_COST = 10;
extern const int DIAGONAL_COST = 14;

const int COST[DIRC_MAX] = {
	10,
	10,
	10,
	10,
	14,
	14,
	14,
	14
};

class gridmap {
public:
	gridmap(int width, int height) {
		m_wgrids = width / GRID_PIXEL;
		m_hgrids = height / GRID_PIXEL;
		//���߽����width,height��������
		m_wgrids++;
		m_hgrids++;
		m_width = width;
		m_height = height;
		m_max_width = m_wgrids * GRID_PIXEL;
		m_max_height = m_hgrids * GRID_PIXEL;

		m_total_grids = m_wgrids * m_hgrids;

		memset(m_neightbuffer, 0, sizeof(grid) * DIRC_MAX);
	}

	~gridmap() {

	}

public:
	int get_grid_num() {
		return m_total_grids;
	}

	const std::vector<grid>& get_all_grids() {
		return m_grids;
	}

	int get_wgrids() {
		return m_wgrids;
	}

	void init() {
		for (int i = 0; i < m_total_grids; i++) {
			int hi = i / m_wgrids; //�������
			int wi = i % m_wgrids;//�������
			m_grids.emplace_back(grid(i, wi, hi));
		}
	}
	
	//�����ķ����赲����
	void build_bocks(int lefttopx, int lefttopy, int rightbottomx, int rightbottomy) {
		if (lefttopx < 0) lefttopx = 0;
		if (lefttopy < 0) lefttopy = 0;

		if (rightbottomx > m_width) rightbottomx = m_width;
		if (rightbottomy > m_height) rightbottomy = m_height;

		if (lefttopx > rightbottomx) return;
		if (lefttopy > rightbottomy) return;

		int lefttopidx = get_grid_index_by_distance_pos(lefttopx, lefttopy);
		grid* lefttop = get_grid(lefttopidx);
		if (!lefttop) return;
		int rightbottomidx = get_grid_index_by_distance_pos(rightbottomx, rightbottomy);
		grid* rightbottom = get_grid(rightbottomidx);
		if (!rightbottom) return;

		for (int x = lefttop->m_x; x <= rightbottom->m_x; x++) {
			for (int y = lefttop->m_y; y <= rightbottom->m_y; y++) {
				int index = get_grid_index_by_grid_pos(x, y);
				assert(index >= 0);
				m_blocks[index] = true;
			}
		}
	}

	int get_grid_index_by_distance_pos(int width, int height) {
		if (width < 0 || width > m_width) return -1;
		if (height < 0 || height > m_height) return -1;

		int w = width / GRID_PIXEL;
		int h = height / GRID_PIXEL;
		//x,y��߽�
		if (w >= m_wgrids) {
			w = m_wgrids - 1;
		}
		if (h >= m_hgrids) {
			h = m_hgrids - 1;
		}
		int index = h * m_wgrids + w;
		return index;
	}

	//ȷ��x,y���ǺϷ���
	int get_grid_index_by_grid_pos(int x, int y) {
		if (x < 0 || x >= m_wgrids) return -1;
		if (y < 0 || y >= m_hgrids) return -1;
		int index = y * m_wgrids + x;
		return index;
	}

	grid* get_grid(int index) {
		if (index < 0 || index >= m_total_grids) return nullptr;
		return &m_grids[index];
	}

	bool is_block(int index) {
		if (index < 0 || index >= m_total_grids) return true;
		if (m_blocks.find(index) != m_blocks.end()) {
			return true;
		}
		return false;
	}

	void calculate_g(grid* cur, const grid* parent) {
		cur->m_g = cur->m_g + parent->m_g;
	}

	void calculate_h(grid* cur, const grid* target) {
		cur->m_h = (std::abs(cur->m_x - target->m_x) + std::abs(cur->m_y - target->m_y)) * STRAIGHT_COST;
		cur->m_f = cur->m_g + cur->m_h;
	}

	//���ص��ھӽ��ĸ���,g��h�����¼���
	grid* get_neighbors(grid* parent) {
		for (int direction = DIRC_LEFT; direction < DIRC_MAX; direction++) {
			grid& g = m_neightbuffer[direction];
			g.m_id = -1; //Ĭ��-1�ǲ�����
			int x = parent->m_x + DIR[direction][0];
			int y = parent->m_y + DIR[direction][1];
			if (!(x >= 0 && x < m_wgrids && y >= 0 && y < m_hgrids)) {
				continue;
			}
			int index = get_grid_index_by_grid_pos(x, y);
			if (is_block(index)) {
				continue;
			}
			grid* tmp = get_grid(index); //:TODO: ������,ֻ�������ж�һ���Ƿ����,�����Ż��� m_grids
			if (tmp) {
				g.m_id = index;
				g.m_x = x;
				g.m_y = y;
				g.clean_weight();
				g.m_g = COST[direction];
				//ֻ�������ھӵ�g, ����ھ��Ѿ���openlist,����������·��,�����¼���f
				calculate_g(&g, parent);
			}
		}
		return m_neightbuffer;
	}
private:
	//��ͼ���
	int m_width;
	int m_max_width;
	int m_height;
	int m_max_height;

	//���(��������)�����ڶ��ٸ���
	int m_wgrids;
	int m_hgrids;
	int m_total_grids;
	//���и���(:TODO: Ϊ������ʾ�����,�������и���,ʵ���Ͽ����Ż���)
	std::vector<grid> m_grids;
	//�赲����
	std::map<int, bool> m_blocks;
	grid m_neightbuffer[DIRC_MAX];
};
