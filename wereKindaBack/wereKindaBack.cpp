#include <cmath>
#include <SFML\Graphics.hpp>
#include <vector>
#include <deque>
#include <algorithm>

const unsigned int screen_width = 1000;
const unsigned int screen_height = 800;

const float gravity = 3600.f;
const float bounciness = 0.95f;
const float friction = 0.99f;

struct Ball {								//ball struct. Just the structure and what defines a ball here
	sf::Vector2f position;
	sf::Vector2f velocity;
	float mass;
	float radius;
	float movement_angle = std::acos(velocity.x);
	sf::CircleShape shape;
	Ball(sf::Vector2f pos, sf::Vector2f vel, float r, float m, sf::Color color)
		: position(pos), velocity(vel), radius(r), mass(m), shape(r)
	{
		shape.setFillColor(color);
	}
};

struct Wall {
	sf::Vector2f start_point;
	sf::Vector2f end_point;
	sf::RectangleShape wall_shape;
	sf::Vector2f diff = end_point - start_point;
	float length = std::sqrt(diff.x * diff.x + diff.y * diff.y);
	float angle = std::atan2(diff.y, diff.x);
	Wall(sf::Vector2f s_pnt, sf::Vector2f e_pnt, float thickness = 3.f)
		: start_point(s_pnt), end_point(e_pnt)
	{
		wall_shape.setSize({ length, thickness });
		wall_shape.setOrigin({0.f, thickness / 2.f});
		wall_shape.setPosition(start_point);
		wall_shape.setRotation(sf::radians(angle));
		wall_shape.setFillColor(sf::Color::White);
	}
};

void step(std::vector<Wall>& walls, std::vector<Ball>& balls, float dt) {					// substeps which happen in between frames. Defines ball movement and collisions

	for (auto& ball : balls) {									// Part of the code calculating the bouncing off of the screen edges
		ball.velocity.y += gravity * dt;
		ball.position += ball.velocity * dt;

		if (ball.position.x > screen_width - 2 * ball.radius) {
			ball.position.x = screen_width - 2 * ball.radius;
			ball.velocity.x *= -bounciness;
			ball.velocity.y *= friction;
		}
		if (ball.position.x < 0) {
			ball.position.x = 0;
			ball.velocity.x *= -bounciness;
			ball.velocity.y *= friction;
		}
		if (ball.position.y > screen_height - 2 * ball.radius) {
			ball.position.y = screen_height - 2 * ball.radius;
			ball.velocity.y *= -bounciness;
			ball.velocity.x *= friction;
		}
		if (ball.position.y < 0) {
			ball.position.y = 0;
			ball.velocity.y *= -bounciness;
			ball.velocity.x *= friction;
		}
	}

	for (std::size_t i = 0; i < balls.size(); i++) {			// Part of the code used to calculate how the balls should hit off of eachother
		for (std::size_t j = i + 1; j < balls.size(); j++) {
			Ball& a = balls[i];
			Ball& b = balls[j];

			sf::Vector2f center_a = a.position + sf::Vector2f(a.radius, a.radius);
			sf::Vector2f center_b = b.position + sf::Vector2f(b.radius, b.radius);
			sf::Vector2f speed_a_temp = a.velocity;
			sf::Vector2f speed_b_temp = b.velocity;

			sf::Vector2f diff = center_b - center_a;					//position difference
			float dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);	//distance
			float min_dist = a.radius + b.radius;

			if (dist < min_dist && dist > 0.f) {
				sf::Vector2f normal = diff / dist;						//normal vector going from a to b
				float overlap = min_dist - dist;
				a.position -= normal * (overlap / 2.f);
				b.position += normal * (overlap / 2.f);

				sf::Vector2f rel = a.velocity - b.velocity;
				float along = rel.x * normal.x + rel.y * normal.y;
				float j = 2 / (a.mass + b.mass);
				float k = (a.velocity.x - b.velocity.x) * (center_a.x - center_b.x) + (a.velocity.y - b.velocity.y) * (center_a.y - center_b.y);
				sf::Vector2f newNormal = (center_a - center_b) / dist;

				if (along > 0.f) {
					speed_a_temp = a.velocity - j * b.mass * (k / dist) * newNormal;
					speed_b_temp = b.velocity + j * a.mass * (k / dist) * newNormal;
					
					a.velocity = speed_a_temp;
					b.velocity = speed_b_temp;
					a.velocity *= bounciness;
					b.velocity *= bounciness;
				}
			}
		}
	}

	for (auto& ball : balls) {
		for (auto& wall : walls) {
			sf::Vector2f center = ball.position + sf::Vector2f(ball.radius, ball.radius);

			sf::Vector2f v_along = wall.diff / wall.length;
			sf::Vector2f v_normal = { -v_along.y, v_along.x };

			sf::Vector2f rel = center - wall.start_point;
			float along_distance = rel.x * v_along.x + rel.y * v_along.y;
			float normal_distance = rel.x * v_normal.x + rel.y * v_normal.y;

			float t = std::clamp(along_distance, 0.f, wall.length);
			sf::Vector2f closest = wall.start_point + v_along * t;
			sf::Vector2f offset = center - closest;
			float dist = std::sqrt(offset.x * offset.x + offset.y * offset.y);

			if (dist < ball.radius && dist > 0.f) {
				sf::Vector2f n = offset / dist;
				ball.position += n * (ball.radius - dist);
				float vn = ball.velocity.x * n.x + ball.velocity.y * n.y;
				if (vn < 0.f) {
					ball.velocity -= n * (vn * (1.f + bounciness));
				}
			}
		}
	}
}

int main() {
	sf::RenderWindow window(sf::VideoMode({ screen_width, screen_height }), "Physics Simulation", sf::Style::Titlebar | sf::Style::Close);
	window.setFramerateLimit(60);

	std::vector<Ball> balls;
	balls.emplace_back(sf::Vector2f{ 25.f, 100.f }, sf::Vector2f{ 300.f, 0.f }, 50.f, 100.f, sf::Color::Red);

	const std::size_t max_history = 4;
	std::deque<sf::Vector2i> mouse_history;

	const int substeps = 8;						// Number of substeps per frame
	sf::Clock clock;

	std::vector<Wall> walls;
	walls.emplace_back(sf::Vector2f{ 100.f, 550.f }, sf::Vector2f{ 275.f, 500.f });
	walls.emplace_back(sf::Vector2f{ 275.f, 500.f }, sf::Vector2f{ 400.f, 550.f });

	while (window.isOpen()) {
		float dt = clock.restart().asSeconds();
		if (dt > 0.05f) dt = 0.05f;

		mouse_history.push_back(sf::Mouse::getPosition(window));
		if (mouse_history.size() > max_history) {
			mouse_history.pop_front();
		}
		float weight_sum = 0.f;									// After this point kinda calculate the mouses velocity
		sf::Vector2f total = { 0.f, 0.f };
		for (std::size_t i = 1; i < mouse_history.size(); i++) {
			sf::Vector2i step_px = mouse_history[i] - mouse_history[i - 1];
			float weight = static_cast<float>(i);
			total.x += step_px.x * weight;
			total.y += step_px.y * weight;
			weight_sum += weight;
		}
		sf::Vector2f mouse_velocity = { 0.f, 0.f };
		if (mouse_history.size() >= 2) {
			mouse_velocity = { total.x / weight_sum * 60.f, total.y / weight_sum * 60.f };
		}

		while (const std::optional event = window.pollEvent()) {	//window polling events like clicking with the mouse or closing the window
			if (event->is<sf::Event::Closed>()) {
				window.close();
			}
			if (const auto* click = event->getIf<sf::Event::MouseButtonPressed>()) {
				if (click->button == sf::Mouse::Button::Left) {
					sf::Vector2i mp = sf::Mouse::getPosition(window);
					float r = 30.f;
					float m = 30.f;
					balls.emplace_back(sf::Vector2f{ mp.x - r, mp.y - r }, mouse_velocity, r, m, sf::Color::Green);
				}
			}
		}

		float sub_dt = dt / substeps;		//applying the step function accordingly
		for (int s = 0; s < substeps; s++) {
			step(walls, balls, sub_dt);
		}

		for (auto& ball : balls) {			//setting all of the balls' positions
			ball.shape.setPosition(ball.position);
		}

		window.clear(sf::Color::Black);		//drawing all of the balls
		for (const auto& ball : balls) {
			window.draw(ball.shape);
		}

		for (const auto& wall : walls) {
			window.draw(wall.wall_shape);
		}

		window.display();					//updating the screen
	}

	return 0;
}