#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "Object.h"
#include "Shader.h"
#include "vertex.h"
#include "texture.h"
#include "camera.h"
#include "read_file.h"
#include "UIManager.h"

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <math.h>
#include <vector>
#include <algorithm>
#include <thread>
#include <random>

using namespace std;

void framebuffer_size_callback(GLFWwindow *window, int width, int height);
void key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);
void processInput(GLFWwindow *window);

int Create3DVoxelTexture();

// settings
const unsigned int SCR_WIDTH = 1000;
const unsigned int SCR_HEIGHT = 750;

// Camera
Camera_c camera(glm::vec3(50.0f, 50.0f, 200.0f));
glm::vec3 lightPos(500.0f, 500.0f, 500.0f);
bool moveObject = 0; // 在移動光源或是相機

// timing
float deltaTime = 0.0f; // time between current frame and last frame
float lastFrame = 0.0f;

ReadFile_c rf("../../vector/1");
UIManager UI;

const int DOMAIN_WIDTH = 128; // 白色背景大小
const int DOMAIN_HEIGHT = 128;
const int DOMAIN_START_X = 20; // 白色背景offset
const int DOMAIN_START_Y = 20;

int grid_size[3] = {64, 128, 256};
unsigned short grid_level0[64 * 64] = {0}, grid_level1[128 * 128] = {0}, grid_level2[256 * 256] = {0};
unsigned short n = 0; // 幾條streamline
unsigned short id = 0; // 幾條streamline
float step = 0.2f;
struct SLposition { // 該條streamline的點座標
    glm::vec2 pos;
    glm::vec2 direction;
    float speed;
    SLposition *last = nullptr;
    SLposition *next = nullptr;
};
struct Streamline {
    unsigned short id; // 第幾條streamline, 從1開始編號
    SLposition *seed = nullptr; // 該條streamline的點座標
    Streamline *next = nullptr; // 下一條streamline
};

// --- 輔助函式：取得指定位置的速度 (包含座標轉換與雙線性內插) ---
glm::vec2 getVelocity(glm::vec2 P) {
    int resX = rf.vec_file.resolution[0];
    int resY = rf.vec_file.resolution[1];
    
    glm::vec2 p_grid = P * static_cast<float>(resX) / static_cast<float>(DOMAIN_WIDTH); // P點轉成資料網格座標
    
    // 取整數部分 (左下)
    int x0 = static_cast<int>(p_grid.x);
    int y0 = static_cast<int>(p_grid.y);
    x0 = std::max(0, std::min(x0, resX - 1));
    y0 = std::max(0, std::min(y0, resY - 1));
    
    // 處裡 (右上) 邊界
    int x1 = std::min(x0 + 1, resX - 1);
    int y1 = std::min(y0 + 1, resY - 1);

    // 取得四個頂點的速度
    glm::vec2 v00 = rf.vec_file.data[rf.idx(x0, y0)];
    glm::vec2 v01 = rf.vec_file.data[rf.idx(x1, y0)];
    glm::vec2 v10 = rf.vec_file.data[rf.idx(x0, y1)];
    glm::vec2 v11 = rf.vec_file.data[rf.idx(x1, y1)];

    // 計算權重
    float tx = p_grid.x - static_cast<float>(x0);
    float ty = p_grid.y - static_cast<float>(y0);

    // 雙線性內插
    return (1 - tx) * (1 - ty) * v00 + 
           (1 - tx) * ty       * v10 + 
           tx       * (1 - ty) * v01 + 
           tx       * ty       * v11;
}

// RK2 method -> 用來依照較準確的速度算下一個點
glm::vec2 RK2(SLposition *sl_pos, float step) {
    glm::vec2 start_pos = sl_pos->pos;
    // 將世界座標轉換到局部座標
    glm::vec2 P0 = start_pos - glm::vec2(DOMAIN_START_X, DOMAIN_START_Y);

    glm::vec2 V0 = getVelocity(P0);
    float speed0 = sqrt(V0.x * V0.x + V0.y * V0.y);
    if(speed0 != 0) V0 = V0 / speed0;

    glm::vec2 P1 = P0 + step * V0;

    glm::vec2 V1 = getVelocity(P1);
    float speed1 = sqrt(V1.x * V1.x + V1.y * V1.y);
    if(speed1 != 0) V1 = V1 / speed1;

    glm::vec2 next_pos = start_pos + step * 0.5f * (V0 + V1);
    
    sl_pos->direction = V0;
    sl_pos->speed = speed0;

    return next_pos;
}

// 如果點太多 or cell被佔 or 速度接近0(critical point) or 遇到邊界就停止
int seeding(SLposition *seed, unsigned short id, unsigned short *mesh, int mesh_size , float step, int max_p_num) {
    int p_num = 1; // 點數量
    SLposition *points = seed;

    // 前進
    while(1) { 
        glm::vec2 next_pos = RK2(points, step);
        if(p_num + 1 > max_p_num) // 點太多
            return p_num;
        if(next_pos.x < DOMAIN_START_X || next_pos.x >= DOMAIN_START_X + DOMAIN_WIDTH || // 遇到邊界
            next_pos.y < DOMAIN_START_Y || next_pos.y >= DOMAIN_START_Y + DOMAIN_HEIGHT)
            break;
        if(points->speed < 0.05) // 速度接近0
            break;
        int cell_i = (next_pos.x - DOMAIN_START_X) * (mesh_size) / DOMAIN_WIDTH;
        int cell_j = (next_pos.y - DOMAIN_START_Y) * (mesh_size) / DOMAIN_HEIGHT;
        int mesh_idx = cell_j * mesh_size + cell_i;
        if(mesh[mesh_idx] != 0 && mesh[mesh_idx] != id) break; // cell被佔
        else mesh[mesh_idx] = id;
        points->next = new SLposition{next_pos};
        points->next->last = points;
        points = points->next;
        p_num++;
    };
    // 後退
    points = seed;
    while(1) { 
        glm::vec2 next_pos = RK2(points, -step);
        if(p_num + 1 > max_p_num)
            return p_num;
        if(next_pos.x < DOMAIN_START_X || next_pos.x >= DOMAIN_START_X + DOMAIN_WIDTH ||
            next_pos.y < DOMAIN_START_Y || next_pos.y >= DOMAIN_START_Y + DOMAIN_HEIGHT)
            break;
        if(points->speed < 0.01)
            break;
        int cell_i = (next_pos.x - DOMAIN_START_X) * (mesh_size) / DOMAIN_WIDTH;
        int cell_j = (next_pos.y - DOMAIN_START_Y) * (mesh_size) / DOMAIN_HEIGHT;
        int mesh_idx = cell_j * mesh_size + cell_i;
        if(mesh[mesh_idx] != 0 && mesh[mesh_idx] != id) break;
        else mesh[mesh_idx] = id;
        points->last = new SLposition{next_pos};
        points->last->next = points;
        points = points->last;
        p_num++;
    };

    return p_num;
}
vector<vector<Vertex_c>> calculateVectorField(float step) {
    struct Streamline *streamline, *first_streamline;
    // initialize
    n = id = 0;
    memset(grid_level0, 0, sizeof(grid_level0));
    memset(grid_level1, 0, sizeof(grid_level1));
    memset(grid_level2, 0, sizeof(grid_level2));
    

    for(int i = 0; i < 128; i++) {
        for(int j = 0; j < 128; j++) {
            if(grid_level0[j * 128 + i] == 0) { // 還沒有被佔位
                n++;
                id++;
                grid_level0[j * 128 + i] = id; // 原點佔位
                // 創建新的streamline
                if(id == 1) { 
                    streamline = new Streamline{id};
                    first_streamline = streamline;
                } else {
                    streamline->next = new Streamline{id};
                    streamline = streamline->next;
                }
                // 網格中間位置
                float x = DOMAIN_START_X + (i + 0.5) * DOMAIN_WIDTH / 128;
                float y = DOMAIN_START_Y + (j + 0.5) * DOMAIN_HEIGHT / 128;
                streamline->seed = new SLposition{glm::vec2(x, y)}; // 加入seed
                int p_num = seeding(streamline->seed, streamline->id, grid_level0, 128, step, 2000);
                if(p_num < 5) { // 太短的清掉
                    SLposition *points = streamline->seed;
                    points = points->next;
                    while(points != nullptr) {
                        SLposition *temp = points;
                        points = points->next;
                        delete(temp);
                    }
                    points = streamline->seed;
                    while(points != nullptr) {
                        SLposition *temp = points;
                        points = points->last;
                        delete(temp);
                    }
                    streamline->seed = nullptr;
                    n--;
                }
            }
        }
    }
    vector<vector<Vertex_c>> sl_vertices;
    vector<Vertex_c> sl_vertices_temp;
    while(first_streamline != nullptr) {
        SLposition *points = first_streamline->seed;
        if(points == nullptr) {
            first_streamline = first_streamline->next;
            continue;
        }
        while(points->last != nullptr) points = points->last;
        while(points->next != nullptr) {
            float speed = points->speed / rf.vec_file.max_speed;
            Vertex_c vertex1{{points->pos.x, points->pos.y, 1.0f}, {0.0f, 0.0f, 0.0f}, {speed}, {}};
            sl_vertices_temp.push_back(vertex1);
            
            SLposition *temp = points->next;
            Vertex_c vertex2{{temp->pos.x, temp->pos.y, 1.0f}, {0.0f, 0.0f, 0.0f}, {speed}, {}};
            sl_vertices_temp.push_back(vertex2);
            points = points->next;
        }
        sl_vertices.push_back(sl_vertices_temp);
        sl_vertices_temp.clear();
        first_streamline = first_streamline->next;
    }
    return sl_vertices;
}


int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    GLFWwindow *window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetKeyCallback(window, key_callback);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // GL 3.0 + GLSL 130
    const char *glsl_version = "#version 130";
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;

    // Setup Dear ImGui style  
    ImGui::StyleColorsDark();
    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // configure global opengl state
    // -----------------------------
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
    Shader_c shader("shader/shader.vs", "shader/shader.fs");
    Shader_c light_shader("shader/light_shader.vs", "shader/light_shader.fs");

    vertex.CreateVertices();
    Object_c square_white;
    square_white.CreateObject(vertices[0], {});
    Object_c square_blue;
    square_blue.CreateObject(vertices[1], {});
    Object_c cube;
    cube.CreateObject(vertices[2], {});
    Object_c axis;  
    axis.CreateObject(vertices[3], {});
    Object_c light_cube;
    light_cube.CreateObject(vertices[4], {});
    
    UI.init();
    vector<vector<Vertex_c>> sl_vertices = calculateVectorField(step);
    Object_c *streamline = new Object_c[n];
    for(int i = 0; i < n; i++) {
        streamline[i].CreateObject(sl_vertices[i], {});
    }
    cout<<n<<endl;
    cout<<id<<endl;

    // render loop
    // -----------
    while (!glfwWindowShouldClose(window))
    {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        // per-frame time logic
        // --------------------
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(window);

        // render
        // ------
        glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui::NewFrame();
        UI.render(lightPos, camera.Position);
        if(UI.isFileUpdata == true) {
            UI.isFileUpdata = false;
            rf.readFile(UI.filename);

            sl_vertices = calculateVectorField(step);
            delete[] streamline;
            streamline = new Object_c[n];
            for(int i = 0; i < n; i++) {
                streamline[i].CreateObject(sl_vertices[i], {});
            }   
            cout<<n<<endl;
            cout<<id<<endl;
        }
        // create model matrix
        glm::mat4 model = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
        // create view matrix
        glm::mat4 view = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first
        // create projection matrix
        glm::mat4 projection = glm::mat4(1.0f); // make sure to initialize matrix to identity matrix first

        // active shader for shading
        shader.use();

        view = camera.GetViewMatrix();
        projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 10000.0f);
        shader.setMat4("view", view);
        shader.setMat4("projection", projection);
        shader.setVec3("lightPos", lightPos);
        shader.setVec3("viewPos", camera.Position);


        // active shader for background
        light_shader.use();
        light_shader.setVec3("lightPos", lightPos);
        light_shader.setMat4("view", view);
        light_shader.setMat4("projection", projection);
        light_shader.setBool("hasTF", false);
        // draw the streamline background
        glBindVertexArray(square_white.VAO_);
        model = glm::mat4(1.0f);
        model = glm::translate(model, glm::vec3(DOMAIN_START_X, DOMAIN_START_Y, 0));
        model = glm::scale(model, glm::vec3(DOMAIN_WIDTH, DOMAIN_HEIGHT, 1));
        light_shader.setMat4("model", model);
        glDrawArrays(GL_TRIANGLES, 0, square_white.size);

        // draw the streamlines
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_1D, UI.getTFTextureID());
        light_shader.setBool("hasTF", true);
        light_shader.setInt("texture0", 0);
        for(int i = 0; i < n; i++) {
            glBindVertexArray(streamline[i].VAO_);
            model = glm::mat4(1.0f);
            light_shader.setMat4("model", model);
            glDrawArrays(GL_LINES, 0, streamline[i].size);
        }
        light_shader.setBool("hasTF", false);

        // draw the light
        glBindVertexArray(light_cube.VAO_);
        model = glm::mat4(1.0f);
        model = glm::translate(model, lightPos);
        model = glm::scale(model, glm::vec3(10, 10, 10));
        light_shader.setMat4("model", model);
        glDrawArrays(GL_TRIANGLES, 0, light_cube.size);
        
        // draw the 3 axis
        glBindVertexArray(axis.VAO_);
        model = glm::mat4(1.0f);
        model = glm::scale(model, glm::vec3(100, 100, 100));
        light_shader.setMat4("model", model);
        glDrawArrays(GL_LINES, 0, axis.size);

        // ImGui render
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // optional: de-allocate all resources once they've outlived their purpose:
    // ------------------------------------------------------------------------

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_Q && action == GLFW_PRESS) {
        moveObject = !moveObject;
    }
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if(moveObject == 0)
    {
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FRONT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            camera.ProcessKeyboard(BACK, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            camera.ProcessKeyboard(LEFT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            camera.ProcessKeyboard(RIGHT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
            camera.ProcessKeyboard(UP, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
            camera.ProcessKeyboard(DOWN, deltaTime);
    }
    else
    {
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            lightPos.z -= 100.0f * deltaTime; 
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            lightPos.z += 100.0f * deltaTime; 
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            lightPos.x -= 100.0f * deltaTime; 
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            lightPos.x += 100.0f * deltaTime; 
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
            lightPos.y += 100.0f * deltaTime; 
        if (glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS)
            lightPos.y -= 100.0f * deltaTime; 
    }

    if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS)
        camera.ProcessKeyboard(PITCHUP, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS)
        camera.ProcessKeyboard(PITCHDOWN, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS)
        camera.ProcessKeyboard(YAWLEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS)
        camera.ProcessKeyboard(YAWRIGHT, deltaTime);

}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow *window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

int Create3DVoxelTexture(vector<unsigned char> voxel_data)
{
    int x = rf.inf_data.data_resolution[0]; 
    int y = rf.inf_data.data_resolution[1];
    int z = rf.inf_data.data_resolution[2]; 

    unsigned char *texture_data = new unsigned char[x * y * z * 4];

    for(int k = 0; k < z; k++)         
        for(int j = 0; j < y; j++)
            for(int i = 0; i < x; i++)  
            {
                int idx = (k * y * x + j * x + i) * 4;
                
                // gradient x
                if(i == 0)
                    texture_data[idx + 0] = (voxel_data[rf.idx(i + 1, j, k)] - voxel_data[rf.idx(i, j, k)]) / rf.inf_data.voxel_size[0];
                else if(i == x - 1)
                    texture_data[idx + 0] = (voxel_data[rf.idx(i, j, k)] - voxel_data[rf.idx(i - 1, j, k)]) / rf.inf_data.voxel_size[0];
                else 
                    texture_data[idx + 0] = (voxel_data[rf.idx(i + 1, j, k)] - voxel_data[rf.idx(i - 1, j, k)]) / (2 * rf.inf_data.voxel_size[0]);

                // gradient y
                if(j == 0)
                    texture_data[idx + 1] = (voxel_data[rf.idx(i, j + 1, k)] - voxel_data[rf.idx(i, j, k)]) / rf.inf_data.voxel_size[1];
                else if(j == y - 1)
                    texture_data[idx + 1] = (voxel_data[rf.idx(i, j, k)] - voxel_data[rf.idx(i, j - 1, k)]) / rf.inf_data.voxel_size[1];
                else 
                    texture_data[idx + 1] = (voxel_data[rf.idx(i, j + 1, k)] - voxel_data[rf.idx(i, j - 1, k)]) / (2 * rf.inf_data.voxel_size[1]);

                // gradient z 
                if(k == 0)
                    texture_data[idx + 2] = (voxel_data[rf.idx(i, j, k + 1)] - voxel_data[rf.idx(i, j, k)]) / rf.inf_data.voxel_size[2];
                else if(k == z - 1)
                    texture_data[idx + 2] = (voxel_data[rf.idx(i, j, k)] - voxel_data[rf.idx(i, j, k - 1)]) / rf.inf_data.voxel_size[2];
                else 
                    texture_data[idx + 2] = (voxel_data[rf.idx(i, j, k + 1)] - voxel_data[rf.idx(i, j, k - 1)]) / (2 * rf.inf_data.voxel_size[2]);

                // intensity
                texture_data[idx + 3] = voxel_data[rf.idx(i, j, k)];
            }

    unsigned int texID;
    glGenTextures(1, &texID);
    glBindTexture(GL_TEXTURE_3D, texID);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // 注意這裡 x 和 z 換位後也要改
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA, x, y, z, 0, GL_RGBA, GL_UNSIGNED_BYTE, texture_data);

    return texID;
}