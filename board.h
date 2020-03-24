#include <cassert>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#define INF 0x3f3f3f3f
#define VISITED -1
#define FINISH 0
#define UNFINISH -2

// ==================================================================
// ==================================================================
// A tiny all-public helper class to record a 2D board position

class Position {
public:
    // the coordinate (-1,-1) is invalid/unitialized
    Position(int r = -1, int c = -1) : row(r), col(c) {}
    int row, col;
};

// convenient functions to print and equality compare Positions
std::ostream& operator<<(std::ostream& ostr, const Position& p);
bool operator==(const Position& a, const Position& b);
bool operator!=(const Position& a, const Position& b);

// ==================================================================
// ==================================================================
// A tiny all-public helper class to store the position & label of a
// robot.  A robot label must be a capital letter.

class Robot {
public:
    Robot(Position p, char w) : pos(p), which(w) {
        assert(isalpha(w) && isupper(w));
    }
    Position pos;
    char which;
};

// ==================================================================
// ==================================================================
// A tiny all-public helper class to store the position & label of a
// goal.  A goal label must be an upper case letter or '?' indicating
// that any robot can occupy this goal to solve the puzzle.

class Goal {
public:
    Goal(Position p, char w) : pos(p), which(w) {
        assert(w == '?' || (isalpha(w) && isupper(w)));
    }
    Position pos;
    char which;
};

// ==================================================================
// ==================================================================
// A class to hold information about the puzzle board including the
// dimensions, the location of all walls, the current position of all
// robots, the goal location, and the robot (if specified) that must
// reach that position

class Board {
public:
    // CONSTRUCTOR
    Board(int num_rows, int num_cols);

    // ACCESSORS related the board geometry
    int getRows() const { return rows; }
    int getCols() const { return cols; }
    bool getHorizontalWall(double r, int c) const;
    bool getVerticalWall(int r, double c) const;

    // ACCESSORS related to the robots and their current positions
    unsigned int numRobots() const { return robots.size(); }
    char getRobot(int i) const {
        assert(i >= 0 && i < (int)numRobots());
        return robots[i].which;
    }
    Position getRobotPosition(int i) const {
        assert(i >= 0 && i < (int)numRobots());
        return robots[i].pos;
    }

    // ACCESSORS related to the overall puzzle goals
    unsigned int numGoals() const { return goals.size(); }
    // (if any robot is allowed to reach the goal, this value is '?')
    char getGoalRobot(int i) const {
        assert(i >= 0 && i < (int)numGoals());
        return goals[i].which;
    }
    Position getGoalPosition(int i) const {
        assert(i >= 0 && i < (int)numGoals());
        return goals[i].pos;
    }

    // MODIFIERS related to board geometry
    void addHorizontalWall(double r, int c);
    void addVerticalWall(int r, double c);

    // MODIFIERS related to robot position
    // initial placement of a new robot
    void placeRobot(const Position& p, char a);
    // move an existing robot
    bool moveRobot(int i, const std::string& direction);

    // void mark_state();

    void restoreRobot(int i, Position pos) {
        Position cur_pos = getRobotPosition(i);
        char robot = getRobot(i);
        setspot(pos, robot);
        robots[i].pos = pos;
        resetspot(cur_pos);
    }

    bool judge_puzzle_finish() {
        for (size_t i = 0; i < goals.size(); i++) {
            Position pos = getGoalPosition(i);
            char goal_robot = getGoalRobot(i);
            char cmp = getspot(pos);
            if (cmp == ' ' || (goal_robot != '?' && goal_robot != cmp)) {
                return false;
            }
        }
        return true;
    }

    // bool same_state();

    int dfs(int moves, int& max_moves) {
        // if (max_moves != -1 && moves > max_moves) {
        //    if (state.count({robots})) {
        //        return state[{robots}];
        //    }
        //    state[{robots}] = UNFINISH;
        //    return UNFINISH;
        //}
        std::cout << "moves = " << moves << std::endl;
        // print();
        if (judge_puzzle_finish()) {
            max_moves = std::min(max_moves, moves);
            std::cout << "bbbb" << std::endl;
            std::cout << "max_moves = " << max_moves << std::endl;
            return FINISH;
        }
        if (state.count({robots}) && state[{robots}] != UNFINISH) {
            std::cout << "aaaa" << std::endl;
            return state[{robots}];
        }
        state[{robots}] = VISITED;
        int next_state;
        int current_state = INF;  // inf
        bool all_is_unfinish = false;
        for (unsigned int i = 0; i < numRobots(); ++i) {
            for (size_t j = 0; j < directions.size(); ++j) {
                Position pos = getRobotPosition(i);
                std::cout << pos.row << " " << pos.col << std::endl;
                if (moveRobot(i, directions[j])) {
                    next_state = dfs(moves + 1, max_moves);
                    if (next_state == VISITED || next_state == UNFINISH) {
                        if (next_state == UNFINISH) {
                            all_is_unfinish = true;
                        }
                        restoreRobot(i, pos);
                        continue;
                    }
                    all_is_unfinish = false;
                    current_state = std::min(current_state, next_state);
                    restoreRobot(i, pos);
                }
            }
        }
        // store_current_state();
        if (all_is_unfinish) {
            printf("nonono\n");
            state[{robots}] = UNFINISH;
            return UNFINISH;
        }
        state[{robots}] = current_state + 1;
        return current_state + 1;
    }
    void pri(int ans) {
        while (ans--) {
            bool pri = true;
            for (unsigned int i = 0; i < numRobots() && pri; ++i) {
                for (size_t j = 0; j < directions.size() && pri; ++j) {
                    Position pos = getRobotPosition(i);
                    if (moveRobot(i, directions[j])) {
                        if(state[{robots}] == ans){
                            print();
                            pri = false;
                            break;
                        }
                        restoreRobot(i, pos);
                    }
                }
            }
        }
    }

    // MODIFIER related to puzzle goals
    void addGoal(const std::string& goal_robot, const Position& p);

    // PRINT
    void print();

private:
    // private helper functions
    char getspot(const Position& p) const;
    void resetspot(const Position& p);
    void setspot(const Position& p, char a);
    char isGoal(const Position& p) const;
    bool getWall(double x, double y);
    bool judge_move(Position cur, Position next) {
        if (next.row < 1 || next.row > rows || next.col < 1 ||
            next.col > cols) {
            return false;
        }
        for (unsigned int i = 0; i < numRobots(); i++) {
            if (robots[i].pos == next) {
                return false;
            }
        }
        double x = (cur.row + next.row) / 2.f;
        double y = (cur.col + next.col) / 2.f;
        return !getWall(x, y);
    }

    // REPRESENTATION

    // the board geometry
    int rows;
    int cols;
    std::vector<std::vector<char> > board;
    std::vector<std::vector<bool> > vertical_walls;
    std::vector<std::vector<bool> > horizontal_walls;

    // my data structure
    static std::vector<std::string> directions;
    struct dir_step {
        int step_x;
        int step_y;
    };
    static std::vector<dir_step> dir_steps;
    struct RobotsPosition {
        std::vector<Robot> robots;
        bool operator==(const RobotsPosition& rsh) const {
            for (size_t i = 0; i < robots.size(); i++) {
                if (robots[i].which != rsh.robots[i].which ||
                    robots[i].pos != rsh.robots[i].pos) {
                    return false;
                }
            }
            return true;
        }
    };
    struct hash_func {
        size_t operator()(const RobotsPosition& robots) const {
            size_t hash_num = 0;
            for (size_t i = 0; i < robots.robots.size(); i++) {
                hash_num += robots.robots[i].pos.row * 10007 +
                            robots.robots[i].pos.col;
                hash_num += (hash_num << 4) ^ (hash_num >> 28) ^
                            robots.robots[i].which;
            }
            return hash_num;
        }
    };
    std::unordered_map<RobotsPosition, int, hash_func> state;

    // the names and current positions of the robots
    std::vector<Robot> robots;

    // the goal positions & the robots that must reach them
    std::vector<Goal> goals;
};
