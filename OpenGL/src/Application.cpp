#include<GL/glew.h>
#include<GLFW/glfw3.h>

#include<iostream>
#include<fstream>
#include<string>
#include<sstream>

#include "Components/Transform.h"
#include "Components/Mesh.h"
#include "Components/DebugVector.h"

#include"../Rendering/Shader.h"
#include"../Managers/Renderer.h"

#include"glm/glm.hpp"
#include"glm/gtc/matrix_transform.hpp"
#include"imgui/imgui.h"
#include"imgui/imgui_impl_glfw_gl3.h"

#include "../Managers/FrameRateController.h"
#include "../Managers/CameraController.h"
#include "../Managers/InputManager.h"
#include "../Managers/Renderer.h"
#include "../Managers/InputManager.h"
#include "../Managers/PhysicsSystem.h"
#include "../Managers/CollisionManager.h"

#include "../GameObject/GameObjectManager.h"
#include "../GameObject/GameObject.h"
#include "../GameObject/ObjectFactory.h"

#include <stack>
#include "GameObject/Profiler.h"

using std::cout;
using std::cin;

GLFWwindow* window; // global window 
FrameRateController* frc;
CameraController* camera;
InputManager* input;
PhysicsSystem* physics;
CollisionManager* colMan;
Renderer* renderer;
GameObjectManager* gpGoManager;
ObjectFactory* objFactory;

float SCR_WIDTH;
float SCR_HEIGHT;

int main(void)
{

	/* Initialize the library */
	if (!glfwInit())
		return -1;
	GLFWmonitor* monitor = glfwGetPrimaryMonitor();
	/*const GLFWvidmode* mode = glfwGetVideoMode(monitor);
	glfwWindowHint(GLFW_RED_BITS, mode->redBits);
	glfwWindowHint(GLFW_GREEN_BITS, mode->greenBits);
	glfwWindowHint(GLFW_BLUE_BITS, mode->blueBits);*/
	//glfwWindowHint(GLFW_REFRESH_RATE, mode->refreshRate);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	SCR_WIDTH = 1280;// (float)mode->width;
	SCR_HEIGHT = 720;// (float)mode->height;
	// change first NULL argument to window to run in full screen mode
	// BEWARE:: if an exception is thrown the window cannot be closed or minimized
	window = glfwCreateWindow((int)SCR_WIDTH, (int)SCR_HEIGHT, "FlyEngine", NULL, NULL);

	/* Create a windowed mode window and its OpenGL context */
	//window = glfwCreateWindow(960, 540, "Hello World", NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		return -1;
	}


	/* Make the window's context current */
	glfwMakeContextCurrent(window);
	glfwSwapInterval(0);//SYNC THE OPENGL WITH THE WINDOW
	// This can only work after creating a valid OpeGL context!!!//VID 3: initializing and binding opengl with glew 
	if (glewInit() != GLEW_OK)
		cout <<" Error in glew init" << std::endl;
	// This can only work after creating a valid OpeGL context!!!
	{
		cout << glGetString(GL_VERSION) << std::endl;
		/* Loop until the user closes the window */
	
		GLCall(glEnable(GL_BLEND));
		GLCall(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
		GLCall(glEnable(GL_DEPTH_TEST));
	
		//Writing down the shader
		Shader shader("res/shaders/Basic.shader");
		Shader debugShader("res/shaders/Debug.shader");

		ImGui::CreateContext();
		ImGui_ImplGlfwGL3_Init(window, true);
		ImGui::StyleColorsDark();

		frc = new FrameRateController(60);
		camera = new CameraController(0, 0, 10.0, 0, 1, 0, -90, 0);
		input = new InputManager();
		gpGoManager = new GameObjectManager();
		renderer = new Renderer();
		objFactory = new ObjectFactory();
		colMan = new CollisionManager();
		physics = new PhysicsSystem();

		//objFactory->LoadLevel("Level0");

		for (int i = 0; i < 100; ++i) {
			gpGoManager->CreateDebugObject();
		}

		bool isPaused = false;
		bool nextStep = false;
		bool isDebugBoundingBox = false;
		bool isDrawColliders = false;
		bool isDebugVectors = false;
		bool isDebugWireframe = false;

		Mesh debugMesh;
		debugMesh.LoadMesh();

		float accumulator = 0.0f;
		float maxPossible_dt = 1.0f / 60.0f; // 60Hz

		// create axes
		/*{
			Transform* tr = gpGoManager->CreateDebugObject(Red);
			tr = gpGoManager->CreateDebugObject(Green);
			tr->Update();
			tr = gpGoManager->CreateDebugObject(Blue);
			tr->Update();
		}*/

		physics->Initialize(); // creates the tree in one shot

		int treeHeight = 0;
		bool drawTree = false;
		int count = 0;
		// Game Loop Starts
		while (!glfwWindowShouldClose(window))
		{
			frc->FrameStart(); // start frame time first

			input->Update(); // update Input first
			ImGui_ImplGlfwGL3_NewFrame();
			// add object on pressing N
			if (input->isPressed(N)) {
				++count;
				GameObject* go = objFactory->LoadObject("Cube");
				Body* pB = static_cast<Body*>(go->GetComponent(BODY));
				Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
				pTr->mPos = glm::vec3(10.0f, 0.0f, 0.0f);
				pB->Initialize();

				physics->dAABBTree.AddCollider(static_cast<Collider*>(go->GetComponent(COLLIDER)));
				//physics->nSquared.AddCollider(static_cast<Collider*>(go->GetComponent(COLLIDER)));
			}

			camera->Update(input, frc->GetFrameTime()); // send updated input to camera

			glm::mat4 proj = glm::perspective(glm::radians(camera->Zoom), 960.0f / 540.0f, 0.1f, 500.0f);
			glm::mat4 view = camera->GetViewMatrix();

			gpGoManager->Update();
			
			isPaused = input->isTriggered(P) == true ? !isPaused : isPaused;
			nextStep = input->isTriggered(F);

			//================Physics Update======================
			float dt = frc->GetFrameTime();


			if (!isPaused) {
				accumulator += dt;
				while (accumulator > maxPossible_dt) {
					//{
						//Timer t;
					physics->Update(maxPossible_dt);
					//}
					accumulator -= maxPossible_dt;
				}
				physics->InterpolateState(accumulator / maxPossible_dt);
			}
			else if (nextStep) {
				accumulator = 0.0f;
				physics->Update(maxPossible_dt);
				physics->InterpolateState(1.0f);
			}
			//====================================================
			
			/* Render here */  
			renderer->Clear();
			if (!isDebugWireframe) {
				shader.Bind();
				{
					for (auto go : gpGoManager->mGameObjects)
					{
						Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
						glm::mat4 mvp = proj * view * pTr->model;

						shader.SetUniformMat4f("u_MVP", mvp);

						Mesh* pM = static_cast<Mesh*>(go->GetComponent(MESH));
						pM->tex->Bind();
						shader.SetUniform1i("u_Texture", 0);
						renderer->Draw(pM->va, shader);
					}
				}
			}
			
			// draw debug
			debugShader.Bind();
			{
				if (isDebugWireframe) {
					for (auto go : gpGoManager->mGameObjects)
					{
						Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
						glm::mat4 mvp = proj * view * pTr->model;

						debugShader.SetUniformMat4f("u_MVP", mvp);
						debugShader.SetUniform4f("u_Color", 1.0f, 1.0f, 0.0f, 1.0f);
						renderer->DebugDraw(debugMesh.va, debugShader);
					}
				}

				if (isDebugVectors) {
					for (auto& c : *colMan->mContacts){
						int i = -1;
						for (auto& cp : c->contactPoints)
						{
							++i;
							GameObject* go = gpGoManager->mDebugObjects[i];
							Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
							pTr->mPos = cp.point;
							pTr->mScale = glm::vec3(0.1f, 0.1f, 0.1f);
							pTr->Update();

							glm::mat4 mvp = proj * view * pTr->model;

							debugShader.SetUniformMat4f("u_MVP", mvp);

							debugShader.SetUniform4f("u_Color", 1.0f, 0.0f, 0.0f, 1.0f);
							renderer->Draw(debugMesh.va, debugShader);
						}
					}
				}
				if (isDrawColliders) {
					for (auto go : gpGoManager->mGameObjects)
					{
						Collider* pCol = static_cast<Collider*>(go->GetComponent(COLLIDER));
						glm::mat4 mvp = proj * view * pCol->mpShape->DebugModelMatrix();

						debugShader.SetUniformMat4f("u_MVP", mvp);

						Mesh* pM = static_cast<Mesh*>(go->GetComponent(MESH));
						debugShader.SetUniform4f("u_Color",
							gpGoManager->Colors[pCol->colliderColor].r,
							gpGoManager->Colors[pCol->colliderColor].g,
							gpGoManager->Colors[pCol->colliderColor].b,
							gpGoManager->Colors[pCol->colliderColor].a);
						renderer->DebugDraw(pM->va, debugShader);
					}
				}
				if (isDebugBoundingBox) {

					Node* root = physics->dAABBTree.GetRoot();

					std::stack<Node *> s;
					Node *curr = root;

					// inorder traversal for printing
					while (curr != NULL || s.empty() == false)
					{
						while (curr != NULL)
						{
							s.push(curr);
							curr = curr->left;
						}

						curr = s.top();
						s.pop();

						glm::mat4 mvp = proj * view * curr->aabb->DebugModelMatrix();

						debugShader.SetUniformMat4f("u_MVP", mvp);
						float color = 0.0f;
						float alpha = 1.0f;
						if(!drawTree) {
							if (curr->height == treeHeight) {
								color = 1.0f;
								debugShader.SetUniform4f("u_Color", color, color, color, alpha);
								renderer->DebugDraw(debugMesh.va, debugShader);
							}
						}
						else {
							color = 1.0f - curr->height / 3.0f;
							debugShader.SetUniform4f("u_Color", color, color, color, alpha);
							renderer->DebugDraw(debugMesh.va, debugShader);
						}

						//debugShader.SetUniform4f("u_Color", color,color,color,alpha);
						//renderer->DebugDraw(debugMesh.va, debugShader);

						curr = curr->right;
					}
				
				}

			}
			
			// iMGui stuff goes here
			{
				// UI Instructions
				ImGui::Text("INSTRUCTIONS:");
				ImGui::Text("Move Camera using WASD");
				ImGui::Text("Right Click and move Mouse to Rotate Camera");
				ImGui::Text("Press P to Pause/Play the Simulation");
				ImGui::Text("Press F to go to Next Frame when Paused");


				int count = gpGoManager->mGameObjects.size();
				if (count < 5) {
					for (int i = 0; i < count; ++i) {
						GameObject* go = gpGoManager->mGameObjects[i];
						//GameObject* go2 = gpGoManager->mGameObjects[2];
						Body* pBody = static_cast<Body*>(go->GetComponent(BODY));
						//Body* pBody2 = static_cast<Body*>(go2->GetComponent(BODY));
						ImGui::PushID(pBody);
						ImGui::SliderFloat3("Velocity ", &pBody->mVel.x, -5.0f, 5.0f);
						ImGui::SliderFloat4("Angular Velocity ", &pBody->mAngularVel.x, -6.0f, 6.0f);
						ImGui::PopID();
						//ImGui::SliderFloat3("box position 2", &pBody2->mPos.x, -10.0f, 10.0f);

					}
				}

				// quaternion test
				{
					//GameObject* go = gpGoManager->mGameObjects[0];
					//ImGui::SliderFloat3("angular velocity", &pBody->mAngularVel.x, -10.0f, 10.0f);
					//Body* pBody = static_cast<Body*>(go->GetComponent(BODY));
					
					//pBody->ApplyForce(glm::vec3(0.0f, 1.0f, 0.0f), pBody->mPos + glm::vec3(0.0f, 0.0f, -0.5f));
				}

				if (physics->dAABBTree.GetRoot()) {
					ImGui::SliderInt("Tree Level", &treeHeight, 0, physics->dAABBTree.GetRoot()->height);
					ImGui::Text("Number of Contacts : %d", physics->dAABBTree.GetPairs().size());
				}
				ImGui::Checkbox("Debug Draw : AABB Tree", &drawTree);
				ImGui::Checkbox("Debug Draw : Bounding Boxes", &isDebugBoundingBox);
				ImGui::Checkbox("Debug Draw : Draw Colliders", &isDrawColliders);
				ImGui::Checkbox("Debug Draw : Contacts", &isDebugVectors);
				ImGui::Checkbox("Debug Draw : Wireframe", &isDebugWireframe);
				ImGui::Checkbox("Apply Friction Impulse", &physics->applyFriction);
				ImGui::SliderInt("Solver Iterations : ", &physics->impulseIterations, 1, 15);

				if (ImGui::Button("Load 1 box")) {
					gpGoManager->mGameObjects.clear();
					physics->applyFriction = true;
					objFactory->LoadLevel("Level1");
					physics->Initialize();
				}
				if (ImGui::Button("Load Stack 3")) {
					gpGoManager->mGameObjects.clear();
					physics->applyFriction = true;
					objFactory->LoadLevel("Level0");
					physics->Initialize();
				} 
				if (ImGui::Button("Load Big Level")) {
					physics->applyFriction = true;
					gpGoManager->mGameObjects.clear();
					objFactory->LoadBigLevel();
					physics->Initialize();
				}
				if (ImGui::Button("Load Ball Joint")) {
					physics->applyFriction = true;
					gpGoManager->mGameObjects.clear();
					objFactory->LoadJointLevel();
					physics->Initialize();
				}
				
				ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
			}

			ImGui::Render();
			ImGui_ImplGlfwGL3_RenderDrawData(ImGui::GetDrawData());
			
			/* Swap front and back buffers */
			glfwSwapBuffers(window);

			/* Poll for and process events */
			glfwPollEvents();

			if (input->isPressed(ESC))
				break;

			frc->FrameEnd();
		}
	}
	ImGui_ImplGlfwGL3_Shutdown();
	ImGui::DestroyContext();
	glfwTerminate();
	return 0;
}