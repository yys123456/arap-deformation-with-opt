#include <iostream>
#include <vector>
#include <algorithm>

namespace BVH {
	using namespace std;
	struct vec3 {
		float x, y, z;
	};

	void out(const vec3 &p) {
		cout << p.x << ' ' << p.y << ' ' << p.z << endl;
	}

	vec3 operator*(const vec3 &a, float b) {
		return { a.x * b, a.y * b, a.z * b };
	}
	vec3 operator+(const vec3 &a, const vec3 &b) {
		return { a.x + b.x , a.y + b.y, a.z + b.z };
	}
	vec3 operator-(const vec3 &a, const vec3 &b) {
		return { a.x - b.x, a.y - b.y, a.z - b.z };
	}
	vec3 operator/(const vec3 &a, float b) {
		return { a.x / b, a.y / b, a.z / b };
	}
	vec3 cross(const vec3 &a, const vec3 &b) {
		return {
			a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x
		};
	}
	float length(const vec3 &a) {
		return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
	}
	vec3 normalize(const vec3 &a) {
		return a / length(a);
	}
	float dot(const vec3 &a, const vec3 &b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}
	struct ray {
		vec3 origin;
		vec3 direction;
		vec3 at(float t) {
			return origin + direction * t;
		}
	};
	struct hit_record {
		float t;
		int face_id;
	};
	struct triangle {
		vec3 p0, p1, p2;
		int id;
		vec3 center() const {
			return (p0 + p1 + p2) / 3.0;
		}
		bool intersect(ray &r, hit_record &rec) {
			vec3 normal = cross(p1 - p0, p2 - p0);
			float down = dot(normal, r.direction);
			if (abs(down) < 1e-5) return 0;
			float t = -(dot(normal, r.origin) - dot(normal, p0)) / down;
			vec3 point = r.at(t);
			vec3 p0p = point - p0, p0p1 = p1 - p0;
			vec3 c0 = cross(p0p1, p0p);
			vec3 p1p = point - p1, p1p2 = p2 - p1;
			vec3 c1 = cross(p1p2, p1p);
			vec3 p2p = point - p2, p2p0 = p0 - p2;
			vec3 c2 = cross(p2p0, p2p);
			if (dot(c0, normal) > 0.0f && dot(c1, normal) > 0.0f && dot(c2, normal) > 0.0f) {
				rec = { t, id };
				return 1;
			}
			return 0;
		}
	};
	struct AABB {
		float minx, miny, minz;
		float maxx, maxy, maxz;
		bool intersect(ray &r) {
			float tx1 = (maxx - r.origin.x) / r.direction.x;
			float tx2 = (minx - r.origin.x) / r.direction.x;
			float ty1 = (maxy - r.origin.y) / r.direction.y;
			float ty2 = (miny - r.origin.y) / r.direction.y;
			float tz1 = (maxz - r.origin.z) / r.direction.z;
			float tz2 = (minz - r.origin.z) / r.direction.z;

			if (tx1 > tx2) swap(tx1, tx2);
			if (ty1 > ty2) swap(ty1, ty2);
			if (tz1 > tz2) swap(tz1, tz2);

			float L = max(max(tx1, ty1), tz1);
			float R = min(min(tx2, ty2), tz2);
			if (L >= R) return 0;
			return 1;
		}
	};
	struct BVHnode {
		BVHnode *lchild, *rchild;
		AABB box;
		int l, r;
	};

	vector<triangle> scene;

	AABB get_box(int l, int r) {
		float minx = 1e18, maxx = -1, miny = 1e18, maxy = -1, minz = 1e18, maxz = -1;
		for (int i = l; i < r; i++) {
			triangle t = scene[i];
			minx = min(minx, t.p0.x);
			minx = min(minx, t.p1.x);
			minx = min(minx, t.p2.x);
			miny = min(miny, t.p0.y);
			miny = min(miny, t.p1.y);
			miny = min(miny, t.p2.y);
			minz = min(minz, t.p0.z);
			minz = min(minz, t.p1.z);
			minz = min(minz, t.p2.z);

			maxx = max(maxx, t.p0.x);
			maxx = max(maxx, t.p1.x);
			maxx = max(maxx, t.p2.x);
			maxy = max(maxy, t.p0.y);
			maxy = max(maxy, t.p1.y);
			maxy = max(maxy, t.p2.y);
			maxz = max(maxz, t.p0.z);
			maxz = max(maxz, t.p1.z);
			maxz = max(maxz, t.p2.z);
		}

		return { minx, miny, minz, maxx, maxy, maxz };
	}

	bool sortX(const triangle &a, const triangle &b) {
		return a.center().x < b.center().x;
	}

	bool sortY(const triangle &a, const triangle &b) {
		return a.center().y < b.center().y;
	}

	bool sortZ(const triangle &a, const triangle &b) {
		return a.center().z < b.center().z;
	}

	BVHnode* dfs(int l, int r) {
		BVHnode *node = new BVHnode();
		node->lchild = NULL;
		node->rchild = NULL;
		node->l = l;
		node->r = r;
		if (r - l < 10) return node;
		AABB box = get_box(l, r);
		float diffx = box.maxx - box.minx;
		float diffy = box.maxy - box.miny;
		float diffz = box.maxz - box.minz;
		node->box = box;
		if (diffx > max(diffy, diffz)) {
			sort(scene.begin() + l, scene.begin() + r + 1, sortX);
			int mid = l + r >> 1;
			node->lchild = dfs(l, mid);
			node->rchild = dfs(mid + 1, r);
		}
		else if (diffy > max(diffx, diffz)) {
			sort(scene.begin() + l, scene.begin() + r + 1, sortY);
			int mid = l + r >> 1;
			node->lchild = dfs(l, mid);
			node->rchild = dfs(mid + 1, r);
		}
		else {
			sort(scene.begin() + l, scene.begin() + r + 1, sortZ);
			int mid = l + r >> 1;
			node->lchild = dfs(l, mid);
			node->rchild = dfs(mid + 1, r);
		}
		return node;
	}

	bool intersect(BVHnode *root, ray &r, hit_record &rec) {
		if (root == NULL) return 0;
		if (root->lchild == NULL && root->rchild == NULL) {
			float minv = 0.0001, maxv = 1e18;
			hit_record t_rec;
			bool hit = false;
			for (int i = root->l; i <= root->r; i++) {
				if (scene[i].intersect(r, t_rec) && t_rec.t >= minv && t_rec.t <= maxv) {
					maxv = min(maxv, t_rec.t);
					rec = t_rec;
					hit = true;
				}
			}
			return hit;
		}
		if (root->box.intersect(r)) {
			hit_record t_rec1, t_rec2;
			bool left_hit = intersect(root->lchild, r, t_rec1);
			bool right_hit = intersect(root->rchild, r, t_rec2);
			if (left_hit && right_hit) {
				rec = t_rec1;
				if (rec.t > t_rec2.t) rec = t_rec2;
				return 1;
			}
			if (left_hit) {
				rec = t_rec1;
				return 1;
			}
			if (right_hit) {
				rec = t_rec2;
				return 1;
			}
			return 0;
		}
		return 0;
	}

	void destroy(BVHnode *root) {
		if (root == NULL) return;
		destroy(root->lchild);
		destroy(root->rchild);
		delete root;
	}

	BVHnode* build() {
		return dfs(0, (int)scene.size() - 1);
	}

	void debug(BVHnode *root) {
		if (!root) return;
		if (root->lchild || root->rchild) {
			cout << "non-leaf ";
		}
		else {
			cout << "leaf ";
		}
		cout << "have primitive: " << root->l << ' ' << root->r << endl;
		debug(root->lchild);
		debug(root->rchild);
	}
};
