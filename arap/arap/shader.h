#pragma once
#include <string>
#include <fstream>
#include <sstream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

struct shader {
	int id;
	shader() {}
	void init(const char* v_path, const char* f_path) {
		std::string v_code;
		std::string f_code;
		std::ifstream v_file;
		std::ifstream f_file;
		v_file.open(v_path);
		f_file.open(f_path);
		std::stringstream v_stream, f_stream;
		v_stream << v_file.rdbuf();
		f_stream << f_file.rdbuf();
		v_file.close();
		f_file.close();
		v_code = v_stream.str();
		f_code = f_stream.str();
		const char* v_shader = v_code.c_str();
		const char* f_shader = f_code.c_str();
		unsigned int vertex, fragment;
		vertex = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(vertex, 1, &v_shader, NULL);
		glCompileShader(vertex);
		fragment = glCreateShader(GL_FRAGMENT_SHADER);
		glShaderSource(fragment, 1, &f_shader, NULL);
		glCompileShader(fragment);
		id = glCreateProgram();
		glAttachShader(id, vertex);
		glAttachShader(id, fragment);
		glLinkProgram(id);
		glDeleteShader(vertex);
		glDeleteShader(fragment);
	}

	void use() {
		glUseProgram(id);
	}

	void set_mat4(const std::string &name, const glm::mat4 &value) {
		int addr = glGetUniformLocation(id, name.data());
		glUniformMatrix4fv(addr, 1, GL_FALSE, glm::value_ptr(value));
	}

	void set_vec4(const std::string &name, const glm::vec4 &value) {
		int addr = glGetUniformLocation(id, name.data());
		glUniform4fv(addr, 1, glm::value_ptr(value));
	}

	void set_vec3(const std::string &name, const glm::vec3 &value) {
		int addr = glGetUniformLocation(id, name.data());
		glUniform3fv(addr, 1, &value[0]);
	}

	void set_int(const std::string &name, int value) {
		int addr = glGetUniformLocation(id, name.data());
		glUniform1i(addr, value);
	}

	void set_float(const std::string &name, float value) {
		int addr = glGetUniformLocation(id, name.data());
		glUniform1f(addr, value);
	}
};