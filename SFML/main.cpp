#include <iostream>
#include <memory>

#include "Common.h"
#include "Application.h"

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
	// Application
	std::shared_ptr<Application> testApp = std::make_shared<Application>();
	testApp->Initialize(window);
	testApp->BuildParticleSystem(PARTICLE_COUNT);

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
						default:
							break;
					}

					break;
				}

				case sf::Event::MouseButtonPressed:
				{
					if (event.key.code == sf::Mouse::Left)
					{
						bLeftMouseClickPressed = true;


					}
					if (event.key.code == sf::Mouse::Right)
					{
						bRightMouseClickPressed = true;
					}
						
					break;
				}
				case sf::Event::MouseButtonReleased:
				{
					if (event.key.code == sf::Mouse::Left)
					{
						bLeftMouseClickPressed = false;
					}
					if (event.key.code == sf::Mouse::Right)
					{
						bRightMouseClickPressed = false;
					}

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
		currentMousePosition = sf::Mouse::getPosition();

		// Application update 
		testApp->Update(time.asSeconds());
		testApp->Draw(window);

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