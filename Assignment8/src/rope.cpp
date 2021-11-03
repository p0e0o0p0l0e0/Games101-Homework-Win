#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {
	Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
	{
		// TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
		Mass* startMass = new Mass(start, node_mass, false);
		Mass* endMass = new Mass(end, node_mass, false);
		Vector2D unitVector = (end - start) / (num_nodes - 1);
		masses.push_back(startMass);
		for (int i = 1; i < num_nodes - 1; i++)
		{
			masses.push_back(new Mass(start + unitVector * i, node_mass, false));
		}
		masses.push_back(endMass);
		for (auto& i : pinned_nodes)
		{
			masses[i]->pinned = true;
		}
		for (int i = 0; i < masses.size() - 1; i++)
		{
			Spring* spring = new Spring(masses[i], masses[i + 1], k);
			springs.push_back(spring);
		}
	}

	void Rope::simulateEuler(float delta_t, Vector2D gravity)
	{
		for (auto& s : springs)
		{
			// TODO (Part 2): Use Hooke's law to calculate the force on a node
			double curLength = (s->m1->position - s->m2->position).norm();
			Vector2D force = s->k * (s->m2->position - s->m1->position) / curLength * (curLength - s->rest_length);
			s->m1->forces += force;
			s->m2->forces -= force;
			// std::cout << "force " << force.x << " " << force.y << std::endl;
		}

		for (auto& m : masses)
		{
			if (!m->pinned)
			{
				// TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
				m->velocity += delta_t * (m->forces + gravity);
				m->position += m->velocity * delta_t;
				std::cout << "m velocity " << m->velocity.x << " " << m->velocity.y << std::endl;
				std::cout << "m position " << m->position.x << " " << m->position.y << std::endl;

				// TODO (Part 2): Add global damping
			}

			// Reset all forces on each mass
			m->forces = Vector2D(0, 0);
		}
	}

	void Rope::simulateVerlet(float delta_t, Vector2D gravity)
	{
		for (auto& s : springs)
		{
			// TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
		}

		for (auto& m : masses)
		{
			if (!m->pinned)
			{
				Vector2D temp_position = m->position;
				// TODO (Part 3.1): Set the new position of the rope mass

				// TODO (Part 4): Add global Verlet damping
			}
		}
	}
}