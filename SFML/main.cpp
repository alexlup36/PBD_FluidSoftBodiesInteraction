#include <iostream>
#include <memory>
#include <limits>

#include "Common.h"
#include "FluidSimulation.h"
#include "SoftBody.h"
#include "SimulationManager.h"

void DrawContainer(sf::RenderWindow& window)
{
	// Draw limits
	sf::Vertex line[] =
	{
		sf::Vertex(sf::Vector2f(WALL_LEFTLIMIT, WALL_TOPLIMIT)), // Top limit
		sf::Vertex(sf::Vector2f(WALL_RIGHTLIMIT, WALL_TOPLIMIT)),

		sf::Vertex(sf::Vector2f(WALL_LEFTLIMIT, WALL_BOTTOMLIMIT)), // Bottom limit
		sf::Vertex(sf::Vector2f(WALL_RIGHTLIMIT, WALL_BOTTOMLIMIT)),

		sf::Vertex(sf::Vector2f(WALL_LEFTLIMIT, WALL_TOPLIMIT)), // Left limit
		sf::Vertex(sf::Vector2f(WALL_LEFTLIMIT, WALL_BOTTOMLIMIT)),

		sf::Vertex(sf::Vector2f(WALL_RIGHTLIMIT, WALL_TOPLIMIT)), // Right limit
		sf::Vertex(sf::Vector2f(WALL_RIGHTLIMIT, WALL_BOTTOMLIMIT)),
	};

	window.draw(line, 8, sf::Lines);
}

void Draw(sf::RenderWindow& window, const sf::Text& stats)
{
	if (FLUID_SIMULATION)
	{
		// Fluid application draw
		std::vector<FluidSimulation*> simulationList = SimulationManager::GetInstance().GetFluidSimulationList();
		for each (FluidSimulation* fluidSim in simulationList)
		{
			fluidSim->Draw(window);
		}
	}

	if (SOFTBODY_SIMULATION)
	{
		// Soft-bodies draw
		std::vector<SoftBody*> SoftBodyList = SimulationManager::GetInstance().GetSoftBodySimulationList();
		for each (SoftBody* pSoftBody in SoftBodyList)
		{
			pSoftBody->Draw(window);
		}
	}

	window.draw(stats);
}

int main()
{
	// ---------------------------------------------------------------------------
	// Window
	bool IsFullScreen = false;
	sf::RenderWindow window(sf::VideoMode(WindowResolution.x, WindowResolution.y), "SFML window");
	window.setFramerateLimit(60);
	sf::Color clearColor = sf::Color::Black;

	// ---------------------------------------------------------------------------
	// Time
	sf::Clock timer;
	sf::Time currentTime = timer.getElapsedTime();
	sf::Time newTime;
	float fTimeAccumulator = 0.0f;
	int iNextGameTick = (int)(SKIP_TICKS * 3.0f);

	// ---------------------------------------------------------------------------
	// Mouse stats
	int iWheelAccumulator = 0;

	sf::Vector2i currentMousePosition = sf::Vector2i();

	// ---------------------------------------------------------------------------
	// Font
	sf::Font font;
	if (!font.loadFromFile("font2.ttf"))
	{
		std::cout << "Failed to load the font." << std::endl;
	}
	sf::Text stats;
	stats.setFont(font);
	stats.setColor(sf::Color::Red);
	stats.setCharacterSize(30);
	stats.setPosition(sf::Vector2f(20.0f, 20.0f));

	// ---------------------------------------------------------------------------
	// Application - fluid implementation
	std::vector<std::shared_ptr<FluidSimulation>> FluidSimulationList;

	if (FLUID_SIMULATION)
	{
		std::shared_ptr<FluidSimulation> fluidSim = std::make_shared<FluidSimulation>();
		fluidSim->BuildParticleSystem(PARTICLE_COUNT);
		FluidSimulationList.push_back(fluidSim);

		// Add the simulation to the list of simulations
		SimulationManager::GetInstance().AddSimulation(fluidSim.get());
	}
	
	// ---------------------------------------------------------------------------
	// Soft-body simulation
	bool bSoftBodyInput = false;

	std::vector<std::shared_ptr<SoftBody>> SoftBodiesList;
	SoftBody* softBodyInstance = nullptr;
	DeformableParticle* deformableControlledParticle = nullptr;
	float fMinimumPickingDistance = 50.0f;
	int iCurrentSBIndex = 0;
	
	// ---------------------------------------------------------------------------

	// Main loop
	while (window.isOpen())
	{
		// Update events
		sf::Event event;
		while (window.pollEvent(event))
		{
			switch (event.type)
			{
				case sf::Event::Closed:
				{
					window.close();
					break;
				}

				case sf::Event::KeyReleased:
				{
					switch (event.key.code)
					{
						// Switch between window and full screen
						case sf::Keyboard::Return:
						{
							if (IsFullScreen)
							{
								window.create(sf::VideoMode(WindowResolution.x, WindowResolution.y), 
									"SFML FullScreen", 
									sf::Style::Default);
								IsFullScreen = false;
							}
							else
							{
								window.create(sf::VideoMode(1920, 1080),
									"SFML FullScreen", 
									sf::Style::Fullscreen);
								IsFullScreen = true;
							}
							break;
						}
						case sf::Keyboard::Escape:
						{
							window.close();

							break;
						}

						// Soft-body creation
						case sf::Keyboard::S:
						{
							if (SOFTBODY_SIMULATION)
							{
								std::cout << "Listening for clicks to add particles in the soft body. Click to add particles in the soft-body." << std::endl;
								bSoftBodyInput = true;

								// Initialize the current soft-body instance
								softBodyInstance = new SoftBody();

								// Add the soft body instance to the list of soft bodies
								//SoftBodiesList.push_back(softBodyInstance);
								SimulationManager::GetInstance().AddSimulation(softBodyInstance);
							}
							
							break;
						}

						// Soft-body reset
						case sf::Keyboard::R:
						{
							if (SOFTBODY_SIMULATION)
							{
								if (softBodyInstance != nullptr)
								{
									// Reset the particle list of the soft-body
									softBodyInstance->ClearSoftBodyParticleList();
								}
							}
							

							break;
						}

						default:
							break;
					}

					break;
				}

				case sf::Event::MouseButtonReleased:
				{
					if (event.key.code == sf::Mouse::Left)
					{
						if (SOFTBODY_SIMULATION)
						{
							// Soft-body mouse control
							if (softBodyInstance != nullptr)
							{
								if (softBodyInstance->IsReady() && deformableControlledParticle)
								{
									// Reset the color of the controlled particle
									deformableControlledParticle->SetDefaultColor();
									// Reset the pointer to the controlled particle
									deformableControlledParticle = nullptr;
								}
							}
						}
					}

					break;
				}

				case sf::Event::MouseButtonPressed:
				{
					if (event.key.code == sf::Mouse::Left)
					{
						// ------------------------------------------------------------------------------------------------
						// Soft body add particle
						if (SOFTBODY_SIMULATION)
						{
							if (bSoftBodyInput)
							{
								// Clamp the click position to the container limits
								currentMousePosition.x = (int)glm::clamp((float)currentMousePosition.x, WALL_LEFTLIMIT, WALL_RIGHTLIMIT);
								currentMousePosition.y = (int)glm::clamp((float)currentMousePosition.y, WALL_TOPLIMIT, WALL_BOTTOMLIMIT);

								int width = 6;
								int height = 6;

								float dt = 0.1f;
								float startPosX = currentMousePosition.x - width * PARTICLE_RADIUS;
								float startPosY = currentMousePosition.y - height * PARTICLE_RADIUS;
								glm::vec2 currentPosition = glm::vec2(startPosX, startPosY);

								for (int i = 0; i < height; i++)
								{
									currentPosition.x = startPosX;

									for (int j = 0; j < width; j++)
									{
										// Create a soft body particle
										DeformableParticle* sbParticle = new DeformableParticle(glm::vec2(currentPosition.x,
											currentPosition.y), softBodyInstance->GetSimulationIndex());

										// Set parent reference
										sbParticle->SetParentRef(softBodyInstance);

										// Add the newly created particle to the soft-body collection
										softBodyInstance->AddSoftBodyParticle(*sbParticle);

										// Update position
										currentPosition.x += (PARTICLE_RADIUS * 2.0f) + dt;
									}

									currentPosition.y += (PARTICLE_RADIUS * 2.0f) + dt;
								}
							}

							// ------------------------------------------------------------------------------------------------
							// Soft-body mouse control
							if (bSoftBodyInput == false)
							{
								glm::vec2 mouseClickPos = glm::vec2(currentMousePosition.x, currentMousePosition.y);

								float fMinimumDistance = std::numeric_limits<float>::max();

								std::vector<SoftBody*> SoftBodyList = SimulationManager::GetInstance().GetSoftBodySimulationList();
								for each (SoftBody* pSoftBody in SoftBodyList)
								{
									// Get the particle list in the current soft-body
									std::vector<DeformableParticle*>& SoftBodyParticleList = pSoftBody->GetParticleList();

									for (unsigned int i = 0; i < SoftBodyParticleList.size(); i++)
									{
										DeformableParticle* currentParticle = SoftBodyParticleList[i];
										float fDistance = glm::length(mouseClickPos - currentParticle->Position);

										if (fDistance < fMinimumDistance)
										{
											fMinimumDistance = fDistance;
											deformableControlledParticle = currentParticle;
											softBodyInstance = pSoftBody;
										}
									}
								}

								if (fMinimumDistance >= fMinimumPickingDistance)
								{
									deformableControlledParticle = nullptr;
								}
								else
								{
									deformableControlledParticle->SetControlledColor();
								}
							}
						}
						
						// ------------------------------------------------------------------------------------------------
					}
					if (event.key.code == sf::Mouse::Right)
					{
						if (SOFTBODY_SIMULATION)
						{
							// End soft body input
							bSoftBodyInput = false;
							softBodyInstance->SetReady(true);

							std::cout << "Soft body input ended. Soft body created." << std::endl;
						}
					}

					// ------------------------------------------------------------------------------------------------
						
					break;
				}

				case sf::Event::MouseWheelMoved:
				{
					iWheelAccumulator += event.mouseWheel.delta;

					std::cout << "Mouse wheel accumulator: " << iWheelAccumulator << std::endl;
				}

				default:
					break;
			}
		}

		// Update

		// Clear the window
		window.clear(clearColor);

		

		// --------------------------------------------------------------------------
		// Get mouse position
		currentMousePosition = sf::Mouse::getPosition(window);

		if (SOFTBODY_SIMULATION)
		{
			// Update the position of the controlled particle
			if (deformableControlledParticle != nullptr)
			{
				deformableControlledParticle->Position = glm::vec2(currentMousePosition.x, currentMousePosition.y);
			}
		}
		
		// Handle simulation time
		newTime = timer.getElapsedTime();
		sf::Time intervalTime = newTime - currentTime;
		int loops = 0;
		float fFrameTime = intervalTime.asSeconds();
		if (fFrameTime > 0.25f)
		{
			fFrameTime = 0.25f;
		}
		currentTime = newTime;
		fTimeAccumulator += fFrameTime;

		while (fTimeAccumulator > iNextGameTick && loops < MAX_FRAMESKIP) 
		{
			for (int speedCounter = 0; speedCounter < SPEEDMULTIPLIER; speedCounter++)
			{
				if (FLUID_SIMULATION)
				{
					// Fluid application update 	
					for each (std::shared_ptr<FluidSimulation> fluidSim in FluidSimulationList)
					{
						fluidSim->Update(window, FIXED_DELTA);
					}
				}
				
				if (SOFTBODY_SIMULATION)
				{
					// Soft-bodies update
					std::vector<SoftBody*> SoftBodyList = SimulationManager::GetInstance().GetSoftBodySimulationList();
					for each (SoftBody* pSoftBody in SoftBodyList)
					{
						pSoftBody->Update(FIXED_DELTA);
					}
				}
				
				std::string fps = "FPS: " + std::to_string(1.0f / intervalTime.asSeconds()) + "\n";
				std::string milisecPerFrame = "Milliseconds per frame: " + std::to_string(intervalTime.asSeconds()) + "\n";
				std::string particleCount = "Particles: " + std::to_string(PARTICLE_COUNT) + "\n";
				std::string gravityStatus = GRAVITY_ON ? "Active" : "Inactive";
				std::string gravityOn = "Gravity: " + gravityStatus + "\n";

				stats.setString(milisecPerFrame + fps + particleCount + gravityOn);
			}

			iNextGameTick += (int)(SKIP_TICKS * 3);
			loops++;
		}

		// Container draw
		DrawContainer(window);
		
		// Draw simulations
		Draw(window, stats);

		// --------------------------------------------------------------------------
		window.display();
	}

	return 0;
}