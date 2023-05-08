/*
Nexus

Copyright(C) 2012 - Federico Ponchio
ISTI - Italian National Research Council - Visual Computing Lab

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License (http://www.gnu.org/licenses/gpl.txt)
for more details.
*/
#include <vcg/space/color4.h>
#include <vcg/math/camera.h>
#include <wrap/system/multithreading/util.h>
#include <wrap/system/time/clock.h>

#include "renderer.h"
#include "nexus.h"
#include "token.h"
#include "controller.h"

#include <QDebug>
#define _USE_MATH_DEFINES
#include <math.h>

extern int current_texture;

using namespace nx;

static struct _UniformLocations
{
    GLint MVP;

    GLint UseNormals;
    GLint UseColors;
    GLint UseTextures;

    GLint LightDir;
    GLint Texture;
} UniformLocations;

const char* vertSrc = R"(
    #version 410

    layout(location = 0) in vec3 a_Position;
    layout(location = 1) in vec3 a_Normal;
    layout(location = 2) in vec4 a_Color;
    layout(location = 3) in vec2 a_TexCoords;

    layout(location = 0) out vec3 v_Normal;
    layout(location = 1) out vec2 v_TexCoords;
    layout(location = 2) out vec4 v_Color;

    uniform mat4 u_MVP;

    void main() {
        gl_Position = u_MVP * vec4(a_Position, 1.0);

        v_Normal = a_Normal;
        v_TexCoords = a_TexCoords;
        v_Color = a_Color;
    }
)";

const char* fragSrc = R"(
    #version 410

    layout(location = 0) in vec3 v_Normal;
    layout(location = 1) in vec2 v_TexCoords;
    layout(location = 2) in vec4 v_Color;

    layout(location = 0) out vec4 Color;

    uniform int u_UseNormals;
    uniform int u_UseColors;
    uniform int u_UseTextures;

    uniform vec3 u_LightDir;
    uniform sampler2D u_Texture;

    void main() {
        vec3 finalColor = vec3(1.0);

        float lighting = 1.0;
        if (u_UseNormals == 1)
            lighting = max(dot(normalize(v_Normal), normalize(-u_LightDir)), 0.0);

        vec4 color = vec4(1.0);
        if (u_UseColors == 1)
            color *= v_Color;

        if (u_UseTextures == 1)
            color *= texture(u_Texture, v_TexCoords);

        finalColor *= color.xyz;
        Color = texture(u_Texture, v_TexCoords);// vec4(max(dot(normalize(v_Normal), normalize(-u_LightDir)), 0.0) * vec3(1.0), 1.0);
    }
)";

/* Da spedire:
 *  - Direzione luce
 *  - Texture
 *
 */

static void checkShaderCompileError(GLuint shader)
{
    GLint compiled;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (compiled == GL_FALSE)
    {
        GLint maxLength = 0;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &maxLength);

        // The maxLength includes the NULL character
        std::vector<GLchar> errorLog(maxLength);
        glGetShaderInfoLog(shader, maxLength, &maxLength, &errorLog[0]);
        std::cout << "Shader compile error: " << std::string(errorLog.data()) << std::endl;
        assert(false);
    }
}

static void checkShaderLinkError(GLuint program)
{
    GLint isLinked = 0;
    glGetProgramiv(program, GL_LINK_STATUS, (int*)&isLinked);
    if (isLinked == GL_FALSE)
    {
        GLint maxLength = 0;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &maxLength);

        // The maxLength includes the NULL character
        std::vector<GLchar> infoLog(maxLength);
        glGetProgramInfoLog(program, maxLength, &maxLength, &infoLog[0]);

        // Use the infoLog as you see fit.
        std::cout << "Shader program failed to link: " << std::string(infoLog.data()) << std::endl;
        assert(false);
    }
}

void Stats::resetAll() {
	//memset(this, 0, sizeof(Stats)); //fast hack.
	rendered = patch_rendered = node_rendered = instance_rendered = 0;
	frustum_culled = cone_culled = occlusion_culled = 0;
	error = instance_error = 0;
}

void Stats::resetCurrent() {
	instance_rendered = 0;
	instance_error = 0.0f;
}

void Renderer::createShader() {
    shader = glCreateProgram();

    GLuint vertShader, fragShader;

    vertShader = glCreateShader(GL_VERTEX_SHADER);
    fragShader = glCreateShader(GL_FRAGMENT_SHADER);

    glShaderSource(vertShader, 1, &vertSrc, nullptr);
    glCompileShader(vertShader);
    glAttachShader(shader, vertShader);
    checkShaderCompileError(vertShader);

    glShaderSource(fragShader, 1, &fragSrc, nullptr);
    glCompileShader(fragShader);
    glAttachShader(shader, fragShader);
    checkShaderCompileError(fragShader);

    glLinkProgram(shader);
    checkShaderLinkError(shader);

    glDeleteShader(vertShader);
    glDeleteShader(fragShader);

    UniformLocations.LightDir = glGetUniformLocation(shader, "u_LightDir");
    UniformLocations.MVP = glGetUniformLocation(shader, "u_MVP");
    UniformLocations.UseColors = glGetUniformLocation(shader, "u_UseColors");
    UniformLocations.UseNormals = glGetUniformLocation(shader, "u_UseNormals");
    UniformLocations.UseTextures = glGetUniformLocation(shader, "u_UseTextures");
    UniformLocations.Texture = glGetUniformLocation(shader, "u_Texture");

    glCheckError();
}


void Renderer::startFrame() {
    stats.resetAll();
	frame++;
}


void Renderer::getView(const float *proj, const float *modelview, const int *viewport) {
	metric.getView(proj, modelview, viewport);
}

void Renderer::nearFar(Nexus *nexus, float &neard, float &fard) {
	
	if(!nexus->isReady()) return;
	
	Frustum &frustum = metric.frustum;
	vcg::Sphere3f s = nexus->header.sphere;
	
	float fd = 0;
	float nd = 1e20;
	frustum.updateNearFar(nd, fd, s.Center(), s.Radius());
	
	//if nd gets too small unnecessarily we lose precision in depth buffer.
	//needs to recourse to find closest but we cheat and just look at the closes vertex in the highest level
	if(nd <= frustum.scale()*s.Radius()/4.0f)
		nd = nexus->nodes[nexus->header.n_nodes-2].error * frustum.scale();
	
	/*        float nnd = 1e20;
		nx::Token *token = nexus->getToken(0);
		if(token->lock()) {
			Node &node = nexus->nodes[0];
			NodeData &data = nexus->data[0];
			vcg::Point3f *points = data.coords();
			for(uint i = 0; i < node.nvert; i++)
				frustum.updateNear(nnd, points[i]);
			token->unlock();
		}
		
		float min_distance = nexus->nodes[0].error * frustum.scale();
		
		if(nd == 1e20)
			nd = min_distance;
		else {
			//if we get closest than the approximation error of the first node we are very close :)
			float approximation = nexus->nodes[0].error * frustum.scale();
			nd -= approximation;
			if(nd < min_distance) nd = min_distance;
		}
	} */
	
	if(nd < neard) neard = nd;
	if(fd > fard) fard = fd;
}

void Renderer::render(Nexus *nexus, vcg::Matrix44f& proj, vcg::Matrix44f& view, bool get_view, int wait) {
    controller = nexus->controller;

    vcg::Point3i viewport;
    glGetIntegerv (GL_VIEWPORT, viewport.V());

    if(!nexus->isReady()) return;
    if(get_view)
        getView(proj.transpose().V(), view.transpose().V(), viewport.V());

	locked.clear();
	last_node = 0;
	
	mt::Clock time = mt::Clock::currentTime();
	time.start();
	if(stats.time.isValid()) {
		int elapsed = stats.time.elapsed();
		if(elapsed > 0) {
			float fps = 1000.0f/elapsed;
			stats.fps = 0.1*fps + 0.9*stats.fps;
		}
	}

	stats.resetCurrent();
	stats.time = time;
	
	if(wait) {
		traverse(nexus);
		
		while(1) {
            if(nexus->controller->isWaiting()) break;
            if(time.elapsed() > wait) break;
			
			mt::sleep_ms(10);
		}
	}

	errors.clear();
	errors.resize(nexus->header.n_nodes);
	traverse(nexus);
	stats.instance_rendered = 0;
	
    Signature &sig = nexus->header.signature;
	bool draw_triangles = sig.face.hasIndex() && (mode & TRIANGLES);
	bool draw_textures = nexus->header.n_textures && (mode & TEXTURES);
	bool draw_texcoords = sig.vertex.hasTextures()&& (mode & TEXTURES);

	if(draw_textures)
		glEnable(GL_TEXTURE_2D);
	if(draw_textures && ! draw_texcoords) {
		glEnable(GL_TEXTURE_2D);
		glEnable(GL_TEXTURE_GEN_S);
		glEnable(GL_TEXTURE_GEN_T);
		glEnable(GL_TEXTURE_GEN_R);
		glEnable(GL_TEXTURE_GEN_Q);
    }

    glUseProgram(shader);
    glUniformMatrix4fv(UniformLocations.MVP, 1, GL_FALSE, (proj * view).transpose().V());
    // [TODO] other uniforms

    renderSelected(nexus);

    // Reset bindings
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    if(draw_triangles) glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

	for(unsigned int i = 0; i < locked.size(); i++)
		locked[i]->unlock();
	locked.clear();
	
	stats.rendered += stats.instance_rendered;
	if(stats.instance_error > stats.error) stats.error = stats.instance_error;
	
	if(draw_textures)
		glDisable(GL_TEXTURE_2D);
	if(draw_textures && ! draw_texcoords) {
		glDisable(GL_TEXTURE_GEN_S);
		glDisable(GL_TEXTURE_GEN_T);
		glDisable(GL_TEXTURE_GEN_R);
		glDisable(GL_TEXTURE_GEN_Q);
	}
	glDisable(GL_COLOR_MATERIAL);
}

void Renderer::endFrame() {
	if(controller)
		controller->endFrame();
	//current_error = stats.total_error;
	//current_rendered = stats.total_rendered;
	
	//porca trottola l'udpdate priorities.
	//non costa molto ma se ci sono un sacco di istance e' uno spreco.
	
	//tenere traccia dei controllers?
}

void Renderer::setMode(Renderer::Mode m, bool on) {
	if(on) mode |= m;
	else mode &= ~m;

    recreateResources = true;
}

void Renderer::renderSelected(Nexus *nexus) {
	Signature &sig = nexus->header.signature;
	bool draw_normals = sig.vertex.hasNormals() && (mode & NORMALS);
	bool draw_colors = sig.vertex.hasColors() && (mode & COLORS ) && !(mode & PATCHES);
	bool draw_triangles = sig.face.hasIndex() && (mode & TRIANGLES);
    bool draw_textures = nexus->header.n_textures && (mode & TEXTURES);
    bool draw_texcoords = sig.vertex.hasTextures()&& (mode & TEXTURES);
	
	/*
	//DEBUG ensure it is a valid cut:
	for(uint32_t i = 0; i < nexus->header.n_nodes-1; i++) {
		Node &node = nexus->nodes[i];
		//if a node is ! selected asser its children are not selected
		if(selected[i]) continue;
		for(uint32_t k = node.first_patch; k < node.last_patch(); k++) {
			Patch &patch = nexus->patches[k];
			assert(!selected[patch.node]);
		}
	}
*/
	int GPU_loaded = 0;
	uint32_t last_texture = 0xffffffff;
    for(uint32_t i = 0; i <= last_node; i++)
    {
		if(!selected[i]) continue;
		
		Node &node = nexus->nodes[i];
		
        if(nexus->header.signature.face.hasIndex() && skipNode(i)) continue;
		stats.node_rendered++;
		
		//TODO cleanup frustum..
		vcg::Sphere3f sphere = node.tightSphere();
		if(frustum_culling && metric.frustum.isOutside(sphere.Center(), sphere.Radius())) {
			stats.frustum_culled++;
			continue;
		}
		if(cone_culling && node.cone.Backface(sphere, metric.frustum.viewPoint())) {
			stats.cone_culled++;
			continue;
		}

        if(0) { //DEBUG
			vcg::Point3f c = sphere.Center();
			float r = node.tight_radius; //sphere.Radius(); //DEBUG
			
			glLineWidth(4);
			glBegin(GL_LINE_STRIP);
			for(int i = 0; i < 21; i++)
				glVertex3f(c[0] + r*sin(i*M_PI/10), c[1] + r*cos(i*M_PI/10), c[2]);
			glEnd();
		}
		
		glCheckError();
		NodeData &data = nexus->nodedata[i];
		assert(data.memory);

        if (recreateResources) {
            GPU_loaded--;
            nexus->dropGpu(i);
            recreateResources = false;
        }
		if(!data.vbo) {
			GPU_loaded++;
            nexus->loadGpu(i, draw_normals, draw_colors, draw_texcoords | draw_textures);
		}
		
		assert(data.vbo);
        assert(data.fbo);
        assert(data.vao);

        glBindVertexArray(data.vao);
        glBindBuffer(GL_ARRAY_BUFFER, data.vbo);

		if(mode & PATCHES) {
			vcg::Color4b  color;
			color[2] = ((i % 11)*171)%63 + 63;
			//color[1] = 255*log2((i+1))/log2(nexus->header.n_patches);
			color[1] = ((i % 7)*57)%127 + 127;
			color[0] = ((i % 16)*135)%127 + 127;
            //metric.getError(node.sphere, node.error, visible); //here we must use the saturated radius.
            glVertexAttrib4f(ATTRIB_COLOR, color[0]/255.0f, color[1]/255.0f, color[2]/255.0f, 1.0f);
        }

		glCheckError();
		
		if(!draw_triangles) {
			//NexusController::instance().splat_renderer.Render(nvert, 6);
			//glPointSize(target_error);
			//float pointsize = 0.4*stats.instance_error;
			float pointsize = ceil(0.3*errors[i]); //2.0; //`0.1 * errors[i];
			//float pointsize = 1.2*stats.instance_error;
			if(pointsize > 2) pointsize = 2;
			glPointSize(pointsize);
			//glPointSize(4*target_error);
			
			glDrawArrays(GL_POINTS, 0, node.nvert);
			stats.patch_rendered++;
			stats.instance_rendered += node.nvert;
			continue;
		}
		
		if(draw_triangles)
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, data.fbo);
		
		uint32_t offset = 0;
		uint32_t end = 0;
		uint32_t last_patch = node.last_patch() - 1;
		for(uint32_t k = node.first_patch; k < node.last_patch(); k++) {
			Patch &patch = nexus->patches[k];
			
			if(!selected[patch.node]) { //we need to draw this
				end = patch.triangle_offset; //advance end
				if(k != last_patch)  //delay rendering
					continue;
			}
            if(end > offset) {
				//TODO GROUP TEXTURES ALSO
				if(draw_textures) {
					if(patch.texture != 0xffffffff) {
						if(last_texture != patch.texture) {
							//if(patch.texture == current_texture) {

							if(draw_textures && !draw_texcoords) {
								Texture &texture = nexus->textures[patch.texture];
								glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
								glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
								glTexGeni(GL_R, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
								glTexGeni(GL_Q, GL_TEXTURE_GEN_MODE, GL_OBJECT_LINEAR);
								
								glTexGenfv(GL_S, GL_OBJECT_PLANE, &texture.matrix[0]);
								glTexGenfv(GL_T, GL_OBJECT_PLANE, &texture.matrix[4]);
								glTexGenfv(GL_R, GL_OBJECT_PLANE, &texture.matrix[8]);
								glTexGenfv(GL_Q, GL_OBJECT_PLANE, &texture.matrix[12]);
							}

							TextureData &tdata = nexus->texturedata[patch.texture];
							glBindTexture(GL_TEXTURE_2D, tdata.tex);
							//glBindTexture(GL_TEXTURE_2D, 0);
                            last_texture = patch.texture;
						}
					} else {
						glBindTexture(GL_TEXTURE_2D, 0);
						last_texture = 0xffffffff;
					}
				}
				
				if(!draw_triangles) {
					if(nexus->header.signature.face.hasIndex())
						glDrawArrays(GL_POINTS, 0, node.nvert);
					else
						glDrawArrays(GL_POINTS, offset, end - offset);
				} else {
					glDrawElements(GL_TRIANGLES, (end - offset)*3, GL_UNSIGNED_SHORT, (void *)(offset*3*sizeof(uint16_t)));
				}
				stats.patch_rendered++;
				stats.instance_rendered += end - offset;
			}
			offset = patch.triangle_offset;
		}
		glCheckError();
        glBindVertexArray(0);
    }
	//	cout << "GPU loaded: " << GPU_loaded <<  endl;
}

Traversal::Action Renderer::expand(HeapNode h) {
	if(h.node > last_node) last_node = h.node;
	
	Nexus *nx = (Nexus *)nexus;
	nx::Token &token = nx->tokens[h.node];
	
	Priority newprio = Priority(h.error, frame);
	Priority oldprio = token.getPriority();
	if(oldprio < newprio)
		token.setPriority(newprio);
	
	nx->controller->addToken(&token);
	
	if(h.node != 0 && (h.error < target_error ||
					   (max_rendered && stats.instance_rendered > max_rendered))) {
		return BLOCK;
	}
	
	Node &node = nx->nodes[h.node];
	if(token.lock()) {
		stats.instance_error = h.error;
		errors[h.node] = h.error;
		if(h.visible) { //TODO check proper working on large models
			bool visible;
			vcg::Sphere3f sphere = node.tightSphere();
			metric.getError(sphere, node.error, visible);
			if(visible)
				stats.instance_rendered += node.nvert/2;
		}
		locked.push_back(&token);
		return EXPAND;
		
	} else {
		return BLOCK;
	}
}


float Renderer::nodeError(uint32_t n, bool &visible) {
	Node &node = ((Nexus *)nexus)->nodes[n];
	return metric.getError(node.sphere, node.error, visible); //here we must use the saturated radius.
	//vcg::Sphere3f sphere = node.tightSphere();
	//return metric.getError(sphere, node.error, visible); //here we must use the saturated radius.
}
