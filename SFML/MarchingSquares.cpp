#include "MarchingSquares.h"

#include "FluidSimulation.h"
#include "BezierCurve.h"

float MarchingSquares::SamplePoint(unsigned int x, unsigned int y)
{
	const std::vector<Particle>& fluidParticleList = FluidSimulation::GetInstance()->GetFluidParticleList();
	// Reset the summation
	float fSum = 0;

	// Iterate through every Metaball in the world 
	for (int i = 0; i < PARTICLE_COUNT; i++)
	{
		fSum += CalculateEquation(fluidParticleList[i], x, y);
	}

	return fSum;
}

void MarchingSquares::ProcessMarchingSquares(sf::RenderWindow& window)
{
	// Go through all the boxes
	for (unsigned int i = 0; i < MAPHEIGHT; i++)
	{
		for (unsigned int j = 0; j < MAPWIDTH; j++)
		{
			// Top left corner pixel coordinates
			sf::Vector2i topLeft = sf::Vector2i(j * BOXSIZE + (int)WALL_LEFTLIMIT, i * BOXSIZE + (int)WALL_TOPLIMIT);
			sf::Vector2i topRight = sf::Vector2i(topLeft.x + BOXSIZE, topLeft.y);
			sf::Vector2i bottomRight = sf::Vector2i(topRight.x, topRight.y + BOXSIZE);
			sf::Vector2i bottomLeft = sf::Vector2i(topLeft.x, topLeft.y + BOXSIZE);

			// Sample the corners for the current sub-division

			// Calculate the samples for the left side of the cell only for the first cell
			// For the next cells copy the values sampled in the previous cell
			if (j > 0)
			{
				m_Map[i][j].SampleTopLeft = m_Map[i][j - 1].SampleTopRight;				// TopLeft
				m_Map[i][j].SampleBottomLeft = m_Map[i][j - 1].SampleBottomRight;			// BottomLeft

				// Copy the intersection points
				m_Map[i][j].LeftEdgeIntersection = m_Map[i][j - 1].RightEdgeIntersection;
			}
			else
			{
				// If we are on the first column but not the first row copy from the prev row
				if (i > 0)
				{
					m_Map[i][j].SampleTopLeft = m_Map[i - 1][j].SampleBottomLeft;					// TopLeft
				}
				else
				{
					m_Map[i][j].SampleTopLeft = SamplePoint(topLeft.x, topLeft.y);				// TopLeft
				}

				// Calculate the sample for the bottom left corner
				m_Map[i][j].SampleBottomLeft = SamplePoint(topLeft.x, topLeft.y + BOXSIZE);		// BottomLeft

				// Calculate the intersection point for the left edge
				m_Map[i][j].LeftEdgeIntersection = glm::vec2((float)topLeft.x, (float)(topLeft.y + (bottomLeft.y - topLeft.y) * ((1.0f - m_Map[i][j].SampleTopLeft) / (m_Map[i][j].SampleBottomLeft - m_Map[i][j].SampleTopLeft))));
			}

			if (i > 0)
			{
				m_Map[i][j].SampleTopRight = m_Map[i - 1][j].SampleBottomRight;							// TopRight

				// Copy the edge intersection from the bottom edge of the top cell
				m_Map[i][j].TopEdgeIntersection = m_Map[i - 1][j].BottomEdgeIntersection;
			}
			else
			{
				m_Map[i][j].SampleTopRight = SamplePoint(topLeft.x + BOXSIZE, topLeft.y);				// TopRight

				// No copy possible as we are in the top cell, calculate the top edge intersection
				m_Map[i][j].TopEdgeIntersection = glm::vec2((float)(topLeft.x + (topRight.x - topLeft.x) * ((1.0f - m_Map[i][j].SampleTopLeft) / (m_Map[i][j].SampleTopRight - m_Map[i][j].SampleTopLeft))), (float)topLeft.y);
			}

			// Calculate the samples for the bottom right corner of the cell every iteration -> no copies possible
			m_Map[i][j].SampleBottomRight = SamplePoint(topLeft.x + BOXSIZE, topLeft.y + BOXSIZE);	// BottomRight

			// Calculate the intersection points for the right and bottom edge -> no copies possible
			m_Map[i][j].RightEdgeIntersection = glm::vec2((float)bottomRight.x, (float)(topRight.y + (bottomRight.y - topRight.y) * ((1.0f - m_Map[i][j].SampleTopRight) / (m_Map[i][j].SampleBottomRight - m_Map[i][j].SampleTopRight))));
			m_Map[i][j].BottomEdgeIntersection = glm::vec2((float)(bottomLeft.x + (bottomRight.x - bottomLeft.x) * ((1.0f - m_Map[i][j].SampleBottomLeft) / (m_Map[i][j].SampleBottomRight - m_Map[i][j].SampleBottomLeft))), (float)bottomLeft.y);

			// Calculate the type of the line
			m_Map[i][j].m_iLineType = (m_Map[i][j].SampleBottomLeft >= 1.0f) |
				((m_Map[i][j].SampleBottomRight >= 1.0f) << 1) |
				((m_Map[i][j].SampleTopRight >= 1.0f) << 2) |
				((m_Map[i][j].SampleTopLeft >= 1.0f) << 3);

			// Exclude the symmetric configurations
			if (m_Map[i][j].m_iLineType > 7)
			{
				m_Map[i][j].m_iLineType = 15 - m_Map[i][j].m_iLineType;
			}

			switch (m_Map[i][j].m_iLineType)
			{
			case 0:
				break;
			case 1:
				BezierCurve::GetInstance().DrawLine(window, m_Map[i][j].LeftEdgeIntersection, m_Map[i][j].BottomEdgeIntersection);
				break;

			case 2:
				BezierCurve::GetInstance().DrawLine(window, m_Map[i][j].BottomEdgeIntersection, m_Map[i][j].RightEdgeIntersection);
				break;

			case 3:
				BezierCurve::GetInstance().DrawLine(window, m_Map[i][j].LeftEdgeIntersection, m_Map[i][j].RightEdgeIntersection);
				break;

			case 4:
				BezierCurve::GetInstance().DrawLine(window, m_Map[i][j].TopEdgeIntersection, m_Map[i][j].RightEdgeIntersection);
				break;

			case 5:
			{
				BezierCurve::GetInstance().DrawLine(window, m_Map[i][j].LeftEdgeIntersection, m_Map[i][j].TopEdgeIntersection);
				BezierCurve::GetInstance().DrawLine(window, m_Map[i][j].BottomEdgeIntersection, m_Map[i][j].RightEdgeIntersection);
				break;
			}

			case 6:
				BezierCurve::GetInstance().DrawLine(window, m_Map[i][j].TopEdgeIntersection, m_Map[i][j].BottomEdgeIntersection);
				break;

			case 7:
				BezierCurve::GetInstance().DrawLine(window, m_Map[i][j].LeftEdgeIntersection, m_Map[i][j].TopEdgeIntersection);
				break;
			}
		}
	}
}