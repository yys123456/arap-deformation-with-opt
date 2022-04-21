#pragma once
#define PI 3.14159265359
extern "C"{
	#include <Opt.h>
};

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <numeric>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/Property.hh>

#include "shader.h"
#include "cuda_runtime.h"
#include "stb_image.h"
#include "bvh.h"

using namespace std;
using namespace OpenMesh;

#define mesh TriMesh_ArrayKernelT<>
#define SELECT 1
#define DEFORM 2
#define INF 1e18

GLFWwindow* window;
const string mesh_path = "../../data/model/Armadillo.obj";
const string mf_path = "./main.fs", mv_path = "./main.vs";
const string wf_path = "./wire.fs", wv_path = "./wire.vs";
const string af_path = "./anchor.fs", av_path = "./anchor.vs";
const string texture_path = "../../data/texture/mat.png";
const int width = 800, height = 800;
float fov = 45, near = 0.1, far = 100;
glm::mat4 model(1.0), drag_rotation(1.0);
glm::vec3 position(0, 0, 3);
glm::vec3 target(0, 0, 0);
glm::vec3 world_up(0, 1, 0);
glm::mat4 view;
glm::mat4 projection;
bool wired = 1, anced = 1;
bool select = 0, drag = 0, deform = 0;
int group_id = -1;
Vec3f offset;
glm::vec2 drag_start;
int select_cnt = 0;
int mode = SELECT;

mesh origin_mesh, result_mesh;
unsigned mesh_vao, mesh_vbo[2];
unsigned texture_id;
int nfaces, nvertices;
shader s_mesh, s_wire, s_anchor;
map<int, vector<pair<int, int>>> on_face;

vector<vector<int>> anchors;
unsigned anc_vao, anc_vbo;
map<int, int> anchor_reg;

Opt_InitializationParameters param;
Opt_State* state;
Opt_Problem* problem;
Opt_Plan* plan;
Vec3f *g_offset, *g_angle, *g_constrains, *g_ref;
unsigned int *g_from, *g_to;
vector<int> c_from, c_to;
int nedges;
float fit_sqrt = 0.25, reg_sqrt = 1;
int non_linear_iteration = 10;
int linear_iteration = 100;

void Opt_solver_init() {
	param = {};
	param.doublePrecision = 0;
	param.verbosityLevel = 1;
	param.collectPerKernelTimingInfo = 1;
	state = Opt_NewState(param);
	problem = Opt_ProblemDefine(state, "./arap.t", "gaussNewtonGPU");
	unsigned int dims[] = { (unsigned int) nvertices };
	plan = Opt_ProblemPlan(state, problem, dims);
	Opt_SetSolverParameter(state, plan, "nIterations", &non_linear_iteration);
	Opt_SetSolverParameter(state, plan, "lIterations", &linear_iteration);
}

void Opt_problem_solve() {
	void* problem_data[] = {
		&fit_sqrt, &reg_sqrt, g_offset, g_angle,
		g_ref, g_constrains, &nedges, g_from, g_to
	};
	Opt_ProblemSolve(state, plan, problem_data);
}

void Opt_problem_delete() {
	Opt_PlanFree(state, plan);
	Opt_ProblemDelete(state, problem);
}

void copy_to_device() {
	cudaMalloc(&g_offset, sizeof(Vec3f) * nvertices);
	cudaMalloc(&g_angle, sizeof(Vec3f) * nvertices);
	cudaMalloc(&g_ref, sizeof(Vec3f) * nvertices);
	cudaMalloc(&g_from, sizeof(int)* nedges);
	cudaMalloc(&g_to, sizeof(int)* nedges);
	cudaMalloc(&g_constrains, sizeof(Vec3f)* nvertices);
	
	vector<Vec3f> vertices;
	for (auto v : origin_mesh.vertices())
		vertices.push_back(origin_mesh.point(v));
	cudaMemcpy(g_ref, vertices.data(), vertices.size() * sizeof(Vec3f), cudaMemcpyHostToDevice);
	cudaMemcpy(g_from, c_from.data(), c_from.size() * sizeof(int), cudaMemcpyHostToDevice);
	cudaMemcpy(g_to, c_to.data(), c_to.size() * sizeof(int), cudaMemcpyHostToDevice);
}

void update_mesh() {
	vector<Vec3f> vertices(nvertices);
	cudaMemcpy(vertices.data(), g_offset, vertices.size() * sizeof(Vec3f), cudaMemcpyDeviceToHost);
	int cnt = 0;
	for (auto v : result_mesh.vertices()) result_mesh.point(v) = vertices[cnt++];
}

void update_constrains(vector<Vec3f> &c_constrains) {
	cudaMemcpy(g_constrains, c_constrains.data(), c_constrains.size() * sizeof(Vec3f), cudaMemcpyHostToDevice);
}

BVH::BVHnode *root = NULL;
void update_bvh() {
	if (root) BVH::destroy(root);
	vector<BVH::triangle> scene;
	for (auto f : result_mesh.faces()) {
		vector<BVH::vec3> p;
		for (auto v : result_mesh.fv_range(f)) {
			auto fv = result_mesh.point(v);
			p.push_back({fv[0], fv[1], fv[2]});
		}
		scene.push_back({ p[0], p[1], p[2], f.idx()});
	}
	BVH::scene = scene;
	root = BVH::build();
}
bool raytracing(float xpos, float ypos, pair<Vec3f, int> &res) {
	double x_ndc, y_ndc;
	x_ndc = 2 * xpos / width - 1.0;
	y_ndc = 1.0 - 2 * ypos / height;
	glm::vec4 clip_pos(x_ndc, y_ndc, 1, 1);
	glm::mat4 inv_mvp = glm::inverse(projection * view * model);
	glm::vec4 model_pos = inv_mvp * clip_pos;
	if (model_pos.w != 0.0) model_pos /= model_pos.w;
	glm::vec4 from = glm::inverse(model) * glm::vec4(position, 1.0);
	glm::vec4 dir = model_pos - from;
	BVH::ray r = {
		{from[0], from[1], from[2]},
		{dir[0], dir[1], dir[2]}
	};
	BVH::hit_record rec;
	if (BVH::intersect(root, r, rec)) {
		BVH::vec3 t_point = r.at(rec.t);
		Vec3f point(t_point.x, t_point.y, t_point.z);
		res = { point, rec.face_id };
		return 1;
	}
	return 0;
}

void update_mesh_buffer() {
	result_mesh.update_face_normals();
	vector<Vec3f> vertices(nfaces * 3);
	vector<pair<Vec3f, int>> t_normals(nvertices, pair<Vec3f, int>(Vec3f(0, 0, 0), 0));
	vector<Vec3f> normals(nfaces * 3);
	int cnt = 0;
	for (auto f : result_mesh.faces()) {
		for (auto v : result_mesh.fv_range(f)) {
			vertices[cnt++] = result_mesh.point(v);
			t_normals[v.idx()].first += result_mesh.normal(f);
			t_normals[v.idx()].second++;
		}
	}
	cnt = 0;
	for (auto f : result_mesh.faces()) {
		for (auto v : result_mesh.fv_range(f)) {
			Vec3f n = t_normals[v.idx()].first / t_normals[v.idx()].second;
			normals[cnt++] = n;
		}
	}

	glBindBuffer(GL_ARRAY_BUFFER, mesh_vbo[0]);
	float *model_vtx_ptr = (float*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	memcpy(model_vtx_ptr, vertices.data(), vertices.size() * sizeof(float) * 3);
	glUnmapBuffer(GL_ARRAY_BUFFER);
	glBindBuffer(GL_ARRAY_BUFFER, mesh_vbo[1]);
	float *model_normal_ptr = (float*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	memcpy(model_normal_ptr, normals.data(), normals.size() * sizeof(float) * 3);
	glUnmapBuffer(GL_ARRAY_BUFFER);
}

void update_anchor_buffer() {
	glBindBuffer(GL_ARRAY_BUFFER, anc_vbo);
	float *anchor_vtx_ptr = (float*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
	//cout << "anchors :" << endl;
	for (int i = 0, cnt = 0; i < anchors.size(); i++) {
		for (auto t : anchors[i]) {
			auto p = result_mesh.point(VertexHandle(t));
			//cout << t << ": " << p << endl;
			anchor_vtx_ptr[cnt++] = p[0];
			anchor_vtx_ptr[cnt++] = p[1];
			anchor_vtx_ptr[cnt++] = p[2];
			anchor_vtx_ptr[cnt++] = i;
		}
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	if (key == GLFW_KEY_D && action == GLFW_RELEASE) {
		mode = DEFORM;
		glfwSetWindowTitle(window, "DEFORM*");
	}
	if (key == GLFW_KEY_S && action == GLFW_RELEASE) {
		mode = SELECT;
		glfwSetWindowTitle(window, "SELECT*");
	}
	if (key == GLFW_KEY_W && action == GLFW_RELEASE) wired ^= 1;
	if (key == GLFW_KEY_A && action == GLFW_RELEASE) anced ^= 1;
	if (key == GLFW_KEY_Z && action == GLFW_RELEASE) {
		if (anchors.size()) {
			for (auto t : anchors.back()) anchor_reg.erase(t);
			anchors.pop_back();
		}
	}
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
	fov -= (float)yoffset;
	if (fov < 1.0f)
		fov = 1.0f;
	if (fov > 180.0f)
		fov = 180.0f;
	projection = glm::perspective(glm::radians(fov), (float)width / (float)height, near, far);
}

int nearest_vertex_id(pair<Vec3f, int> &res, bool in_group) {
	float minv = INF;
	int v_id = -1;
	FaceHandle f = FaceHandle(res.second);
	for (auto v : result_mesh.fv_range(f)) {
		auto p = result_mesh.point(v);
		auto d = res.first - p;
		float dist = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
		if (dist < minv && (in_group ? anchor_reg.count(v.idx()) : 1)) {
			minv = dist;
			v_id = v.idx();
		}
	}
	return v_id;
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
	if (button == GLFW_MOUSE_BUTTON_LEFT) {
		if (action == GLFW_PRESS) {
			drag = true;
			double xpos, ypos;
			glfwGetCursorPos(window, &xpos, &ypos);
			drag_start = glm::vec2(xpos, ypos);
		}
		else if (action == GLFW_RELEASE) {
			drag = false;
			model = drag_rotation * model;
			drag_rotation = glm::mat4(1.0f);
		}
	}
	else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
		if (mode == SELECT) {
			if (action == GLFW_PRESS) {
				select = true;
				anchors.push_back(vector<int>());
			}
			else if (action == GLFW_RELEASE) {
				select = false;
				if (anchors.back().size() == 0) {
					anchors.pop_back();
					return;
				}
				update_anchor_buffer();
			}
		}
		else if (mode == DEFORM) {
			if (action == GLFW_PRESS) {
				double xpos, ypos;
				glfwGetCursorPos(window, &xpos, &ypos);
				pair<Vec3f, int> res;
				if (raytracing(xpos, ypos, res)) {
					int v_id = nearest_vertex_id(res, 1);
					if (v_id == -1) return;
					group_id = anchor_reg[v_id];
					drag_start = glm::vec2(xpos, ypos);
					deform = true;
				}
			}
			else if (action == GLFW_RELEASE) {
				if (deform) {
					deform = false;
					for (auto i : anchors[group_id])
						result_mesh.point(VertexHandle(i)) += offset;
					group_id = -1;
					vector<Vec3f> c_constrains(nvertices, Vec3f(-INF, -INF, -INF));
					for (auto g : anchors) {
						for (auto i : g) {
							auto p = VertexHandle(i);
							c_constrains[i] = result_mesh.point(p);
						}
					}
					update_constrains(c_constrains);
					// solve
					Opt_problem_solve();
					update_mesh();
					update_bvh();
					update_mesh_buffer();
					update_anchor_buffer();
				}
			}
		}
	}
}
void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
	if (drag) {
		glm::mat3 to_world = glm::inverse(glm::mat3(view));
		glm::vec2 drag_vec = glm::vec2(xpos - drag_start.x, drag_start.y - ypos);
		glm::vec3 axis_vec = glm::normalize(to_world * glm::vec3(-drag_vec.y, drag_vec.x, 0));
		float angle = glm::length(drag_vec) / height / 2 * PI;
		drag_rotation = glm::rotate(glm::mat4(1.0f), angle, axis_vec);
	}
	if (select) {
		pair<Vec3f, int> res;
		if (raytracing(xpos, ypos, res)) {
			int v_id = nearest_vertex_id(res, 0);
			if (anchor_reg.count(v_id)) return;
			anchor_reg[v_id] = (int) anchors.size() - 1;
			anchors.back().push_back(v_id);
		}
	}
	if (deform) {
		glm::mat3 to_model = glm::inverse(glm::mat3(view * model));
		glm::vec2 drag_vec = glm::vec2(xpos - drag_start.x, drag_start.y - ypos);
		glm::vec3 offset_vec = to_model * glm::vec3(drag_vec.x, drag_vec.y, 0);
		
		offset = Vec3f(offset_vec[0], offset_vec[1], offset_vec[2]) * 0.01;
		glBindBuffer(GL_ARRAY_BUFFER, mesh_vbo[0]);
		float *model_vtx_ptr = (float*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
		for (auto i : anchors[group_id]) {
			auto p = result_mesh.point(VertexHandle(i));
			for (auto j : on_face[i]) {
				model_vtx_ptr[j.first * 9 + j.second * 3] = p[0] + offset[0];
				model_vtx_ptr[j.first * 9 + j.second * 3 + 1] = p[1] + offset[1];
				model_vtx_ptr[j.first * 9 + j.second * 3 + 2] = p[2] + offset[2];
			}
		}
		glUnmapBuffer(GL_ARRAY_BUFFER);
		glBindBuffer(GL_ARRAY_BUFFER, anc_vbo);
		float *anc_vtx_ptr = (float*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
		int start = 0;
		for (int i = 0; i < group_id; i++) start += anchors[i].size();
		for(int i = 0; i < anchors[group_id].size(); i ++) {
			auto p = result_mesh.point(VertexHandle(anchors[group_id][i]));
			anc_vtx_ptr[(start + i) * 4] = p[0] + offset[0];
			anc_vtx_ptr[(start + i) * 4 + 1] = p[1] + offset[1];
			anc_vtx_ptr[(start + i) * 4 + 2] = p[2] + offset[2];
		}
		glUnmapBuffer(GL_ARRAY_BUFFER);	
	}
}


void opengl_init() {
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	window = glfwCreateWindow(width, height, "SELECT*", nullptr, nullptr);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetCursorPosCallback(window, cursor_position_callback);
	glfwSetKeyCallback(window, key_callback);
	glfwSetScrollCallback(window, scroll_callback);
	if (window == nullptr) {
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
	}
	glfwMakeContextCurrent(window);
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout << "Failed to initialize GLAD" << std::endl;
	}
	// shader init
	s_mesh.init(mv_path.c_str(), mf_path.c_str());
	s_wire.init(wv_path.c_str(), wf_path.c_str());
	s_anchor.init(av_path.c_str(), af_path.c_str());

	glGenTextures(1, &texture_id);
	glBindTexture(GL_TEXTURE_2D, texture_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	int w, h, c;
	//stbi_set_flip_vertically_on_load(true);
	unsigned char *data = stbi_load(texture_path.c_str(), &w, &h, &c, 0);
	if (c == 3) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
	if (c == 4) glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

	glGenerateMipmap(GL_TEXTURE_2D);
	stbi_image_free(data);

	glGenBuffers(2, mesh_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, mesh_vbo[0]);
	glBufferData(GL_ARRAY_BUFFER, origin_mesh.n_faces() * 9 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, mesh_vbo[1]);
	glBufferData(GL_ARRAY_BUFFER, origin_mesh.n_faces() * 9 * sizeof(float), nullptr, GL_DYNAMIC_DRAW);

	glGenVertexArrays(1, &mesh_vao);
	glBindVertexArray(mesh_vao);
	glBindBuffer(GL_ARRAY_BUFFER, mesh_vbo[0]);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
	glBindBuffer(GL_ARRAY_BUFFER, mesh_vbo[1]);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glGenBuffers(1, &anc_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, anc_vbo);
	glBufferData(GL_ARRAY_BUFFER, origin_mesh.n_vertices() * sizeof(float) * 4, nullptr, GL_DYNAMIC_DRAW);

	glGenVertexArrays(1, &anc_vao);
	glBindVertexArray(anc_vao);
	glBindBuffer(GL_ARRAY_BUFFER, anc_vbo);
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), nullptr);
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glClearColor(0, 0, 0, 1);
	glEnable(GL_DEPTH_TEST);
	glPointSize(10);

	view = glm::lookAt(glm::vec3(0, 0, 3), glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
	projection = glm::perspective(glm::radians(fov), (float)width / (float)height, near, far);
}

void draw() {
	glfwPollEvents();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	s_mesh.use();
	s_mesh.set_int("matcap_sampler", 0);
	s_mesh.set_mat4("model", drag_rotation * model);
	s_mesh.set_mat4("projection", projection);
	s_mesh.set_mat4("view", view);
	
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, texture_id);
	glBindVertexArray(mesh_vao);
	glDrawArrays(GL_TRIANGLES, 0, nfaces * 3);

	if (wired) {
		s_wire.use();
		s_wire.set_mat4("model", drag_rotation * model);
		s_wire.set_mat4("projection", projection);
		s_wire.set_mat4("view", view);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glBindVertexArray(mesh_vao);
		glDrawArrays(GL_TRIANGLES, 0, nfaces * 3);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
	if (anced) {
		s_anchor.use();
		s_anchor.set_mat4("model", drag_rotation * model);
		s_anchor.set_mat4("projection", projection);
		s_anchor.set_mat4("view", view);
		glBindVertexArray(anc_vao);
		int sum = 0;
		for (auto t : anchors) sum += t.size();
		//cout << sum << endl;
		glDrawArrays(GL_POINTS, 0, sum);
	}
	glfwSwapBuffers(window);
}

void geometry_to_origin(mesh &m) {
	float total_area = 0;
	Vec3f center(0, 0, 0);
	for (auto f : m.faces()) {
		vector<Vec3f> p;
		for (auto v : m.fv_range(f)) 
			p.push_back(m.point(v));
		Vec3f c = (p[0] + p[1] + p[2]) / 3.0f;
		float area = 0.5 * cross(p[1] - p[0], p[2] - p[0]).length();
		center += c * area;
		total_area += area;
	}
	center /= total_area;
	for (auto v : m.vertices()) {
		auto &p = m.point(v);
		p -= center;
	}
}

void out(glm::mat4 &m) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++)
			cout << m[i][j] << ' ';
		cout << endl;
	}
}

int main() {
	if (!OpenMesh::IO::read_mesh(origin_mesh, mesh_path) || !OpenMesh::IO::read_mesh(result_mesh, mesh_path)) {
		cout << "read error";
		return 0;
	}
	geometry_to_origin(origin_mesh);
	geometry_to_origin(result_mesh);
	origin_mesh.request_face_normals();
	result_mesh.request_face_normals();
	nvertices = origin_mesh.n_vertices();
	nfaces = origin_mesh.n_faces();

	for (auto f : origin_mesh.faces()) {
		int cnt = 0;
		for (auto v : origin_mesh.fv_range(f))
			on_face[v.idx()].push_back({ f.idx(), cnt++ });
	}

	for (auto v : origin_mesh.vertices()) {
		for (auto vv : origin_mesh.vv_range(v)) {
			c_from.push_back(v.idx());
			c_to.push_back(vv.idx());
		}
	}
	nedges = c_from.size();
	
	Opt_solver_init();
	copy_to_device();

	opengl_init();
	update_mesh_buffer();
	update_anchor_buffer();
	update_bvh();

	out(model);
	out(view);
	out(projection);

	while (!glfwWindowShouldClose(window)) draw();
	glfwTerminate();
}

