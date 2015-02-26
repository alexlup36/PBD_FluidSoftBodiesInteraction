#include <iostream>
#include <memory>
#include <limits>

#include "Common.h"
#include "Application.h"
#include "SoftBody.h"

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

int main()
{
	// ---------------------------------------------------------------------------
	// Window
	bool bIsFullScreen = false;

	sf::Vector2i windowResolution = sf::Vector2i(1280, 800);
	sf::Vector2i fullscreenResolution = sf::Vector2i(1920, 1080);

	sf::RenderWindow window(sf::VideoMode(windowResolution.x, windowResolution.y), "SFML window");
	//window.setFramerateLimit(60);
	sf::Color clearColor = sf::Color::Black;

	// ---------------------------------------------------------------------------
	// Time
	sf::Clock timer;
	sf::Time time;
	
	// ---------------------------------------------------------------------------
	// Mouse stats
	bool bLeftMouseClickPressed = false;
	bool bRightMouseClickPressed = false;
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
	//std::shared_ptr<Application> testApp = std::make_shared<Application>();
	//testApp->Initialize(window);
	//testApp->BuildParticleSystem(PARTICLE_COUNT);

	// ---------------------------------------------------------------------------
	// Soft-body simulation
	bool bSoftBodyInput = false;

	std::vector<std::shared_ptr<SoftBody>> SoftBodiesList;
	std::shared_ptr<SoftBody> softBodyInstance = nullptr;
	SoftBodyParticle* softBodyControlledParticle = nullptr;
	float fMinimumPickingDistance = 50.0f;
	int iCurrentSBIndex = 0;

	// ---------------------------------------------------------------------------
	// Main loop
	while (window.isOpen())
	{
		// Restart timer
		timer.restart().asSeconds();

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
							if (bIsFullScreen)
							{
								window.create(sf::VideoMode(windowResolution.x, windowResolution.y), 
									"SFML FullScreen", 
									sf::Style::Default);
								bIsFullScreen = false;
							}
							else
							{
								window.create(sf::VideoMode(fullscreenResolution.x, fullscreenResolution.y),
									"SFML FullScreen", 
									sf::Style::Fullscreen);
								bIsFullScreen = true;
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
							std::cout << "Listening for clicks to add particles in the soft body. Click to add particles in the soft-body." << std::endl;
							bSoftBodyInput = true;

							// Initialize the current soft-body instance
							softBodyInstance = std::make_shared<SoftBody>();
							// Add the soft body instance to the list of soft bodies
							SoftBodiesList.push_back(softBodyInstance);

							break;
						}

						// Soft-body reset
						case sf::Keyboard::R:
						{
							if (softBodyInstance != nullptr)
							{
								// Reset the particle list of the soft-body
								softBodyInstance->ClearSoftBodyParticleList();
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
						bLeftMouseClickPressed = false;

						// Soft-body mouse control
						if (softBodyInstance != nullptr)
						{
							if (softBodyInstance->IsReady() && softBodyControlledParticle)
							{
								// Reset the color of the controlled particle
								softBodyControlledParticle->SetDefaultColor();
								// Reset the pointer to the controlled particle
								softBodyControlledParticle = nullptr;
							}
						}
					}
					if (event.key.code == sf::Mouse::Right)
					{
						bRightMouseClickPressed = false;
					}

					break;
				}

				case sf::Event::MouseButtonPressed:
				{
					if (event.key.code == sf::Mouse::Left)
					{
						bLeftMouseClickPressed = true;

						// ------------------------------------------------------------------------------------------------
						// Soft-body mouse control
						if (bSoftBodyInput == false)
						{
							glm::vec2 mouseClickPos = glm::vec2(currentMousePosition.x, currentMousePosition.y);

							float fMinimumDistance = std::numeric_limits<float>::max();
							for each (std::shared_ptr<SoftBody> softBody in SoftBodiesList)
							{
								// Get the particle list in the current soft-body
								std::vector<SoftBodyParticle>& SoftBodyParticleList = softBody->GetParticleList();

								for (unsigned int i = 0; i < SoftBodyParticleList.size(); i++)
								{
									SoftBodyParticle& currentParticle = SoftBodyParticleList[i];
									float fDistance = glm::length(mouseClickPos - currentParticle.Position);

									if (fDistance < fMinimumDistance)
									{
										fMinimumDistance = fDistance;
										softBodyControlledParticle = &currentParticle;
										softBodyInstance = softBody;
									}
								}
							}

							if (fMinimumDistance >= fMinimumPickingDistance)
							{
								softBodyControlledParticle = nullptr;
							}
							else
							{
								softBodyControlledParticle->SetControlledColor();
							}
						}
						
						// ------------------------------------------------------------------------------------------------
					}
					if (event.key.code == sf::Mouse::Right)
					{
						bRightMouseClickPressed = true;
					}

					// ------------------------------------------------------------------------------------------------
					// Soft body add particle

					if (bLeftMouseClickPressed && bSoftBodyInput)
					{
						// Clamp the click position to the container limits
						currentMousePosition.x = (int)glm::clamp((float)currentMousePosition.x, WALL_LEFTLIMIT, WALL_RIGHTLIMIT);
						currentMousePosition.y = (int)glm::clamp((float)currentMousePosition.y, WALL_TOPLIMIT, WALL_BOTTOMLIMIT);
						
						// Create a soft body particle
						SoftBodyParticle sbParticle = SoftBodyParticle(glm::vec2(currentMousePosition.x,
							currentMousePosition.y));

						// Add the newly created particle to the soft-body collection
						softBodyInstance->AddSoftBodyParticle(sbParticle);
					}

					// ------------------------------------------------------------------------------------------------

					if (bRightMouseClickPressed)
					{
						// End soft body input
						bSoftBodyInput = false;
						softBodyInstance->SetReady(true);

						std::cout << "Soft body input ended. Soft body created." << std::endl; 
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

		// Update the position of the controlled particle
		if (softBodyControlledParticle != nullptr)
		{
			softBodyControlledParticle->Position = glm::vec2(currentMousePosition.x, currentMousePosition.y);
		}

		// Fluid application update 
		//testApp->Update(window, time.asSeconds());
		// Soft-bodies update
		for each (std::shared_ptr<SoftBody> softBody in SoftBodiesList)
		{
			softBody->Update(time.asSeconds());
		}

		// Container draw
		DrawContainer(window);
		// Fluid application draw
		//testApp->Draw(window);
		// Soft-bodies draw
		for each (std::shared_ptr<SoftBody> softBody in SoftBodiesList)
		{
			softBody->Draw(window);
		}

		std::string fps = "FPS: " + std::to_string(1.0f / time.asSeconds()) + "\n";
		std::string milisecPerFrame = "Milliseconds per frame: " + std::to_string(time.asSeconds()) + "\n";
		std::string particleCount = "Particles: " + std::to_string(PARTICLE_COUNT) + "\n";
		std::string gravityStatus = GRAVITY_ON ? "Active" : "Inactive";
		std::string gravityOn = "Gravity: " + gravityStatus + "\n";

		stats.setString(milisecPerFrame + fps + particleCount + gravityOn);
		window.draw(stats);

		// --------------------------------------------------------------------------
		window.display();

		time = timer.getElapsedTime();
	}

	return 0;
}