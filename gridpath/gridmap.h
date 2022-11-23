#pragma once
#include "grid.h"
#include<cstdlib>
#include<vector>
#include<map>
#include<cassert>

// 二维地图格子边长(多少个像素大小)
const int GRID_PIXEL = 30;

//格子邻居的8个方向
//直线方向
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


// DIRC_MAX 个方向以及方向对应的开销
const int DIR[DIRC_MAX][2] = {
	{-1,0},
	{1,0},
	{0,-1}, //这里注意一下,默认当前坐标系(0,0)是在左上角,x轴向右延申,y轴向下延申
	{0,1},
	{-1,-1},
	{ -1,1 },
	{ 1,-1 },
	{ 1,1 }
};

const int* get_dir(int direction) {
	return DIR[direction];
}

//F = G + H, 这是对应 DIRC_MAX 个方向邻居的 G 开销
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
		//最大边界包括width,height的整数倍
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
			int hi = i / m_wgrids; //深度坐标
			int wi = i % m_wgrids;//宽度坐标
			m_grids.emplace_back(grid(i, wi, hi));
		}
	}
	
	//创建四方形阻挡区域
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
		//x,y轴边界
		if (w >= m_wgrids) {
			w = m_wgrids - 1;
		}
		if (h >= m_hgrids) {
			h = m_hgrids - 1;
		}
		int index = h * m_wgrids + w;
		return index;
	}

	//确保x,y都是合法的
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

	//返回的邻居结点的副本,g和h都重新计算
	grid* get_neighbors(grid* parent) {
		for (int direction = DIRC_LEFT; direction < DIRC_MAX; direction++) {
			grid& g = m_neightbuffer[direction];
			g.m_id = -1; //默认-1是不存在
			int x = parent->m_x + DIR[direction][0];
			int y = parent->m_y + DIR[direction][1];
			if (!(x >= 0 && x < m_wgrids && y >= 0 && y < m_hgrids)) {
				continue;
			}
			int index = get_grid_index_by_grid_pos(x, y);
			if (is_block(index)) {
				continue;
			}
			grid* tmp = get_grid(index); //:TODO: 在这里,只是用来判断一下是否存在,考虑优化掉 m_grids
			if (tmp) {
				g.m_id = index;
				g.m_x = x;
				g.m_y = y;
				g.clean_weight();
				g.m_g = COST[direction];
				//只计算新邻居的g, 如果邻居已经在openlist,并且是优先路径,再重新计算f
				calculate_g(&g, parent);
			}
		}
		return m_neightbuffer;
	}
private:
	//地图宽高
	int m_width;
	int m_max_width;
	int m_height;
	int m_max_height;

	//宽高(横向竖向)各存在多少格子
	int m_wgrids;
	int m_hgrids;
	int m_total_grids;
	//所有格子(:TODO: 为方便演示和理解,创建所有格子,实际上可以优化掉)
	std::vector<grid> m_grids;
	//阻挡格子
	std::map<int, bool> m_blocks;
	grid m_neightbuffer[DIRC_MAX];
};
