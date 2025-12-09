/**
 * @file micromouse_node.cpp
 * @brief MicroMouse ROS2 node with DFS navigation
 *
 * Complete the TODOs to implement the ROS2 node
 *
 * Algorithm: Depth-First Search (DFS) with dynamic replanning.
 * - Uses a stack (LIFO) to explore paths
 * - Does NOT guarantee shortest path - finds any valid path
 * - Neighbor priority order: North, East, South, West
 * - Replans when walls are discovered that block the current path
 *
 * Point values are shown in each TODO comment.
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "micromouse_cpp/maze_control_api.hpp"
#include "micromouse_interfaces/action/navigate_to_goal.hpp"
#include "micromouse_interfaces/srv/get_robot_status.hpp"

using namespace std::chrono_literals;
using NavigateToGoal = micromouse_interfaces::action::NavigateToGoal;
using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateToGoal>;
using GetRobotStatus = micromouse_interfaces::srv::GetRobotStatus;

namespace micromouse {

/**
 * @brief MicroMouse ROS2 Node
 *
 * Provides:
 * - Action server: /navigate_to_goal
 * - Service: /get_robot_status
 * - Publisher: /robot_position
 *
 * Can run in standalone mode (immediate navigation) or wait for action client.
 */
class MicroMouseNode : public rclcpp::Node {
 public:
  MicroMouseNode() : Node("micromouse_node") {
    // =====================================================================
    // TODO 1 (5 points): Declare and retrieve parameters
    // =====================================================================
    // Declare 5 parameters with default values:
    //   - "goal_x" (int, default: 7)
    //   - "goal_y" (int, default: 7)
    //   - "path_color" (string, default: "c")
    //   - "goal_color" (string, default: "g")
    //   - "standalone_mode" (bool, default: true)
    //
    // Then retrieve each parameter into the corresponding member variable.
    // For string parameters, get the first character: .as_string()[0]
    // =====================================================================
    // YOUR CODE HERE

    // =====================================================================
    // TODO 2 (5 points): Create publisher for robot position
    // =====================================================================
    // Create a publisher for geometry_msgs::msg::Point on topic
    // "/robot_position" with a queue size of 10. Store in robot_position_pub_.
    // =====================================================================
    // YOUR CODE HERE

    // =====================================================================
    // TODO 3 (5 points): Create service server for robot status
    // =====================================================================
    // Create a service server for GetRobotStatus on "/get_robot_status".
    // Bind to get_status_callback method.
    // Store in status_srv_.
    // =====================================================================
    // YOUR CODE HERE

    // =====================================================================
    // TODO 4 (10 points): Create action server for navigation
    // =====================================================================
    // Create an action server for NavigateToGoal on "/navigate_to_goal".
    // Bind three callbacks:
    //   - handle_goal (goal request callback)
    //   - handle_cancel (cancel request callback)
    //   - handle_accepted (goal accepted callback)
    // Store in action_server_.
    // =====================================================================
    // YOUR CODE HERE

    log("Action server ready: /navigate_to_goal");
  }

  /**
   * @brief Run navigation immediately (standalone MMS mode)
   */
  void run_navigation() {
    W_ = MazeControlAPI::get_maze_width();
    H_ = MazeControlAPI::get_maze_height();

    log("Maze: " + std::to_string(W_) + "x" + std::to_string(H_) + ", Goal: (" +
        std::to_string(goal_x_) + ", " + std::to_string(goal_y_) + ")");

    init_walls();

    Cell start{0, 0};
    Cell goal{goal_x_, goal_y_};

    MazeControlAPI::set_text(goal.x, goal.y, "G");
    MazeControlAPI::set_color(goal.x, goal.y, goal_color_);

    execute_with_replanning(start, goal);
  }

  /**
   * @brief Check if running in standalone mode
   * @return true if standalone mode (immediate navigation), false if waiting
   * for action
   */
  bool is_standalone_mode() const {
    return standalone_mode_;
  }

 private:
  // Maze dimensions
  int W_{16};
  int H_{16};
  int goal_x_{7};
  int goal_y_{7};

  // Visualization colors
  char path_color_{'c'};
  char goal_color_{'g'};

  // Mode
  bool standalone_mode_{true};

  // Internal wall representation: walls_[x][y][dir] = true if wall exists
  std::vector<std::vector<std::array<bool, 4>>> walls_;

  // Track explored cells
  std::set<Cell> explored_cells_;

  // Robot state
  Cell robot_{0, 0};
  Dir facing_{Dir::North};
  int steps_{0};

  // Timing
  std::chrono::steady_clock::time_point nav_start_time_;
  bool is_running_{false};
  std::mutex state_mutex_;

  // ROS2 interfaces
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr robot_position_pub_;
  rclcpp::Service<GetRobotStatus>::SharedPtr status_srv_;
  rclcpp_action::Server<NavigateToGoal>::SharedPtr action_server_;

  // =========================================================================
  // Utility Methods (PROVIDED)
  // =========================================================================

  void log(const std::string& msg) {
    RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
    std::cerr << "[LOG] " << msg << std::endl;
  }

  bool in_bounds(const Cell& c) const {
    return c.x >= 0 && c.y >= 0 && c.x < W_ && c.y < H_;
  }

  // =========================================================================
  // Wall Management (PROVIDED)
  // =========================================================================

  /**
   * @brief Initialize internal wall storage and set perimeter walls
   */
  void init_walls() {
    walls_.assign(
        W_, std::vector<std::array<bool, 4>>(H_, {false, false, false, false}));

    // Set perimeter walls
    for (int x = 0; x < W_; ++x) {
      walls_[x][0][static_cast<int>(Dir::South)] = true;
      MazeControlAPI::set_wall(x, 0, 's');
      walls_[x][H_ - 1][static_cast<int>(Dir::North)] = true;
      MazeControlAPI::set_wall(x, H_ - 1, 'n');
    }
    for (int y = 0; y < H_; ++y) {
      walls_[0][y][static_cast<int>(Dir::West)] = true;
      MazeControlAPI::set_wall(0, y, 'w');
      walls_[W_ - 1][y][static_cast<int>(Dir::East)] = true;
      MazeControlAPI::set_wall(W_ - 1, y, 'e');
    }
  }

  /**
   * @brief Mark wall on both sides (current cell and neighbor)
   */
  void mark_wall(const Cell& c, Dir d) {
    if (!in_bounds(c))
      return;

    walls_[c.x][c.y][static_cast<int>(d)] = true;
    MazeControlAPI::set_wall(c.x, c.y, dir_char(d));

    Cell nb{c.x + dx(d), c.y + dy(d)};
    if (in_bounds(nb)) {
      walls_[nb.x][nb.y][static_cast<int>(opposite(d))] = true;
      MazeControlAPI::set_wall(nb.x, nb.y, dir_char(opposite(d)));
    }
  }

  /**
   * @brief Sense walls and update internal map
   */
  void sense_and_update(const Cell& pos, Dir facing) {
    std::cerr << "[DEBUG] Sensing at (" << pos.x << "," << pos.y << ") facing "
              << dir_to_string(facing) << std::endl;

    if (MazeControlAPI::has_wall_front()) {
      mark_wall(pos, facing);
    }

    if (MazeControlAPI::has_wall_right()) {
      Dir d = static_cast<Dir>((static_cast<int>(facing) + 1) % 4);
      mark_wall(pos, d);
    }

    if (MazeControlAPI::has_wall_left()) {
      Dir d = static_cast<Dir>((static_cast<int>(facing) + 3) % 4);
      mark_wall(pos, d);
    }
  }

  /**
   * @brief Check if edge between cell and direction is free (no wall)
   */
  bool edge_free(const Cell& c, Dir d) const {
    if (!in_bounds(c))
      return false;
    if (walls_[c.x][c.y][static_cast<int>(d)])
      return false;

    Cell n{c.x + dx(d), c.y + dy(d)};
    if (!in_bounds(n))
      return false;
    if (walls_[n.x][n.y][static_cast<int>(opposite(d))])
      return false;

    return true;
  }

  // =========================================================================
  // Movement (PROVIDED)
  // =========================================================================

  /**
   * @brief Turn robot to face desired direction
   */
  void face_direction(Dir want) {
    int h = static_cast<int>(facing_);
    int w = static_cast<int>(want);
    int delta = (w - h + 4) % 4;

    if (delta == 1) {
      MazeControlAPI::turn_right();
      facing_ = want;
    } else if (delta == 2) {
      MazeControlAPI::turn_right();
      MazeControlAPI::turn_right();
      facing_ = want;
    } else if (delta == 3) {
      MazeControlAPI::turn_left();
      facing_ = want;
    }
  }

  // =========================================================================
  // DFS Path Planning
  // =========================================================================

  /**
   * @brief Compute path using Depth-First Search
   *
   * Uses iterative DFS with a stack. Explores neighbors in order:
   * North, East, South, West. Does NOT guarantee shortest path.
   *
   * @param start Starting cell
   * @param goal Target cell
   * @return Path from start to goal, or nullopt if no path exists
   */
  std::optional<std::vector<Cell>> dfs_plan(const Cell& start, const Cell& goal) {
        std::vector<Cell> stack;
        std::set<Cell> visited;
        std::map<Cell, Cell> parent;
        
        stack.push_back(start);
        parent[start] = start;
        
        while (!stack.empty()) {
            Cell cur = stack.back();
            stack.pop_back();
            
            if (visited.count(cur)) continue;
            visited.insert(cur);
            
            if (cur == goal) {
                // Reconstruct path
                std::vector<Cell> path;
                for (Cell at = cur; !(at == parent[at]); at = parent[at]) {
                    path.push_back(at);
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }
            
            // Check neighbors: N, E, S, W priority
            std::array<std::pair<Cell, Dir>, 4> neighbors = {{
                {{cur.x, cur.y + 1}, Dir::North},
                {{cur.x + 1, cur.y}, Dir::East},
                {{cur.x, cur.y - 1}, Dir::South},
                {{cur.x - 1, cur.y}, Dir::West}
            }};
            
            for (const auto& [nxt, d] : neighbors) {
                if (!in_bounds(nxt)) continue;
                if (!edge_free(cur, d)) continue;
                if (!visited.count(nxt) && !parent.count(nxt)) {
                    parent[nxt] = cur;
                }
                if (!visited.count(nxt)) {
                    stack.push_back(nxt);
                }
            }
        }
        
        return std::nullopt;
    }

  // =========================================================================
  // Visualization (PROVIDED)
  // =========================================================================

  void color_path(const std::vector<Cell>& path, const Cell& goal) {
    MazeControlAPI::clear_all_color();
    for (size_t i = 0; i < path.size(); ++i) {
      MazeControlAPI::set_color(path[i].x, path[i].y, path_color_);
      MazeControlAPI::set_text(path[i].x, path[i].y, std::to_string(i));
    }
    // Re-color goal cell after clearing
    MazeControlAPI::set_color(goal.x, goal.y, goal_color_);
  }

  // =========================================================================
  // Main Navigation Loop (PROVIDED)
  // =========================================================================

  /**
   * @brief Execute navigation with dynamic replanning
   *
   * Algorithm:
   * 1. Sense walls at current position
   * 2. Plan path using DFS
   * 3. Follow path, sensing before each move
   * 4. If wall blocks path, replan from current position
   * 5. Repeat until goal reached or no path exists
   *
   * @param start Starting cell
   * @param goal Target cell
   * @param goal_handle Optional action goal handle for feedback (nullptr for
   * standalone)
   * @return true if goal reached, false otherwise
   */
  bool execute_with_replanning(
      const Cell& start, const Cell& goal,
      std::shared_ptr<GoalHandleNavigate> goal_handle = nullptr) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      is_running_ = true;
      nav_start_time_ = std::chrono::steady_clock::now();
    }

    robot_ = start;
    facing_ = Dir::North;
    steps_ = 0;
    explored_cells_.clear();
    explored_cells_.insert(robot_);

    // Create feedback message if using action
    std::shared_ptr<NavigateToGoal::Feedback> feedback = nullptr;
    if (goal_handle) {
      feedback = std::make_shared<NavigateToGoal::Feedback>();
    }

    MazeControlAPI::set_text(robot_.x, robot_.y, "S");

    // Initial sense
    sense_and_update(robot_, facing_);

    // Initial plan
    auto plan = dfs_plan(robot_, goal);
    if (!plan) {
      log("Initial DFS failed - no path to goal.");
      std::lock_guard<std::mutex> lock(state_mutex_);
      is_running_ = false;
      return false;
    }

    log("Path found with " + std::to_string(plan->size()) + " cells");
    color_path(*plan, goal);
    MazeControlAPI::set_color(robot_.x, robot_.y, 'y');

    size_t idx = 1;
    int replans = 0;

    while (!(robot_ == goal)) {
      // Check for cancellation if using action
      if (goal_handle && goal_handle->is_canceling()) {
        log("Navigation cancelled by client");
        std::lock_guard<std::mutex> lock(state_mutex_);
        is_running_ = false;
        return false;
      }

      if (idx >= plan->size()) {
        log("Replanning from (" + std::to_string(robot_.x) + "," +
            std::to_string(robot_.y) + ")");
        replans++;
        plan = dfs_plan(robot_, goal);
        if (!plan) {
          log("No path found from current position.");
          std::lock_guard<std::mutex> lock(state_mutex_);
          is_running_ = false;
          return false;
        }
        log("Path found with " + std::to_string(plan->size()) + " cells");
        color_path(*plan, goal);
        MazeControlAPI::set_color(robot_.x, robot_.y, 'y');
        idx = 1;
      }

      Cell next = (*plan)[idx];

      // Determine required direction
      Dir want = facing_;
      if (next.x == robot_.x && next.y == robot_.y + 1) {
        want = Dir::North;
      } else if (next.x == robot_.x + 1 && next.y == robot_.y) {
        want = Dir::East;
      } else if (next.x == robot_.x && next.y == robot_.y - 1) {
        want = Dir::South;
      } else if (next.x == robot_.x - 1 && next.y == robot_.y) {
        want = Dir::West;
      } else {
        log("Planned step is not adjacent; triggering replanning.");
        replans++;
        plan = dfs_plan(robot_, goal);
        if (!plan) {
          log("No path found from current position.");
          std::lock_guard<std::mutex> lock(state_mutex_);
          is_running_ = false;
          return false;
        }
        color_path(*plan, goal);
        MazeControlAPI::set_color(robot_.x, robot_.y, 'y');
        idx = 1;
        continue;
      }

      // Turn toward target and sense BEFORE moving
      face_direction(want);
      sense_and_update(robot_, facing_);

      // If wall detected in front, replan
      if (!edge_free(robot_, facing_)) {
        log("Wall detected on planned edge; replanning.");
        replans++;
        plan = dfs_plan(robot_, goal);
        if (!plan) {
          log("No path found after encountering wall.");
          std::lock_guard<std::mutex> lock(state_mutex_);
          is_running_ = false;
          return false;
        }
        log("Path found with " + std::to_string(plan->size()) + " cells");
        color_path(*plan, goal);
        MazeControlAPI::set_color(robot_.x, robot_.y, 'y');
        idx = 1;
        continue;
      }

      // Move forward
      MazeControlAPI::move_forward();
      robot_.x += dx(facing_);
      robot_.y += dy(facing_);
      steps_++;
      explored_cells_.insert(robot_);

      MazeControlAPI::set_color(robot_.x, robot_.y, 'y');
      sense_and_update(robot_, facing_);
      publish_position();

      // Publish feedback if using action
      if (goal_handle && feedback) {
        feedback->current_x = robot_.x;
        feedback->current_y = robot_.y;
        feedback->direction = static_cast<uint8_t>(facing_);
        feedback->elapsed_seconds = get_elapsed_seconds();
        goal_handle->publish_feedback(feedback);

        RCLCPP_DEBUG(this->get_logger(),
                     "Feedback: pos=(%d,%d), dir=%d, time=%.2f",
                     feedback->current_x, feedback->current_y,
                     feedback->direction, feedback->elapsed_seconds);

        // Small delay to allow executor to transmit feedback
        std::this_thread::sleep_for(10ms);
      }

      // Advance along plan if consistent
      if (idx < plan->size() && robot_ == (*plan)[idx]) {
        ++idx;
      } else {
        log("Deviation detected; replanning.");
        replans++;
        plan = dfs_plan(robot_, goal);
        if (!plan) {
          log("Lost plan and no alternative found.");
          std::lock_guard<std::mutex> lock(state_mutex_);
          is_running_ = false;
          return false;
        }
        color_path(*plan, goal);
        MazeControlAPI::set_color(robot_.x, robot_.y, 'y');
        idx = 1;
      }
    }

    log("Reached goal in " + std::to_string(steps_) + " steps with " +
        std::to_string(replans) + " replans!");
    MazeControlAPI::set_color(goal.x, goal.y, goal_color_);
    MazeControlAPI::set_text(goal.x, goal.y, "GOAL");

    std::lock_guard<std::mutex> lock(state_mutex_);
    is_running_ = false;
    return true;
  }

  // =========================================================================
  // ROS2 Interface Callbacks
  // =========================================================================

  void publish_position() {
    auto msg = geometry_msgs::msg::Point();
    msg.x = static_cast<double>(robot_.x);
    msg.y = static_cast<double>(robot_.y);
    msg.z = static_cast<double>(facing_);
    robot_position_pub_->publish(msg);
  }

  double get_elapsed_seconds() const {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(now - nav_start_time_).count();
  }

  void get_status_callback(
      const std::shared_ptr<GetRobotStatus::Request> /*request*/,
      std::shared_ptr<GetRobotStatus::Response> response) {
    // =====================================================================
    // TODO 6 (10 points): Populate service response with robot status
    // =====================================================================
    // Fill in all 9 response fields:
    //   - position_x, position_y: robot_.x, robot_.y
    //   - direction: use dir_to_string(facing_)
    //   - steps_taken: steps_
    //   - steps_to_goal_estimate: Manhattan distance to goal
    //     (use std::abs(goal_x_ - robot_.x) + std::abs(goal_y_ - robot_.y))
    //   - elapsed_seconds: call get_elapsed_seconds() if running, else 0.0
    //   - is_running: is_running_
    //   - success: true (service call succeeded)
    //   - message: "Navigation active" if running, else "Idle"
    //
    // Note: Use std::lock_guard<std::mutex> lock(state_mutex_) for thread
    // safety
    // =====================================================================
    std::lock_guard<std::mutex> lock(state_mutex_);
    // YOUR CODE HERE
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& /*uuid*/,
      std::shared_ptr<const NavigateToGoal::Goal> goal) {
    log("Received goal: (" + std::to_string(goal->goal_x) + ", " +
        std::to_string(goal->goal_y) + ")");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleNavigate> /*goal_handle*/) {
    log("Cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle) {
    std::thread{
        std::bind(&MicroMouseNode::execute_action, this, std::placeholders::_1),
        goal_handle}
        .detach();
  }

  void execute_action(const std::shared_ptr<GoalHandleNavigate> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<NavigateToGoal::Result>();

    goal_x_ = goal->goal_x;
    goal_y_ = goal->goal_y;

    log("Executing action: navigate to (" + std::to_string(goal_x_) + ", " +
        std::to_string(goal_y_) + ")");

    W_ = MazeControlAPI::get_maze_width();
    H_ = MazeControlAPI::get_maze_height();
    init_walls();

    Cell start{0, 0};
    Cell goal_cell{goal_x_, goal_y_};

    // Set goal color before navigation
    MazeControlAPI::set_text(goal_cell.x, goal_cell.y, "G");
    MazeControlAPI::set_color(goal_cell.x, goal_cell.y, goal_color_);

    // Execute with goal_handle for feedback
    bool success = execute_with_replanning(start, goal_cell, goal_handle);

    result->success = success;
    result->total_steps = steps_;
    result->total_time = get_elapsed_seconds();
    result->message = success ? "Goal reached!" : "No path found";

    if (goal_handle->is_canceling()) {
      result->message = "Navigation cancelled";
      goal_handle->canceled(result);
      log("Action canceled");
    } else if (success) {
      goal_handle->succeed(result);
      log("Action succeeded");
    } else {
      goal_handle->abort(result);
      log("Action aborted");
    }
  }
};

}  // namespace micromouse

// =============================================================================
// Main
// =============================================================================

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<micromouse::MicroMouseNode>();

  if (node->is_standalone_mode()) {
    // Run navigation in separate thread so ROS2 services still work
    std::thread nav_thread([node]() {
      node->run_navigation();
      rclcpp::shutdown();
    });

    rclcpp::spin(node);

    if (nav_thread.joinable()) {
      nav_thread.join();
    }
  } else {
    // Use MultiThreadedExecutor to allow feedback publishing while action
    // executes
    RCLCPP_INFO(node->get_logger(), "Waiting for action client...");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }

  rclcpp::shutdown();
}
