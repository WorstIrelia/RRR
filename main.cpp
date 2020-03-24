#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "board.h"

// ==================================================================================
// ==================================================================================

// This function is called if there was an error with the command line arguments
bool usage(const std::string& executable_name) {
    std::cerr << "Usage: " << executable_name << " <puzzle_file>" << std::endl;
    std::cerr << "       " << executable_name << " <puzzle_file> -max_moves <#>"
              << std::endl;
    std::cerr << "       " << executable_name << " <puzzle_file> -all_solutions"
              << std::endl;
    std::cerr << "       " << executable_name
              << " <puzzle_file> -visualize <which_robot>" << std::endl;
    std::cerr << "       " << executable_name
              << " <puzzle_file> -max_moves <#> -all_solutions" << std::endl;
    std::cerr << "       " << executable_name
              << " <puzzle_file> -max_moves <#> -visualize <which_robot>"
              << std::endl;
    exit(0);
}

// ==================================================================================
// ==================================================================================

// load a Ricochet Robots puzzle from the input file
Board load(const std::string& executable, const std::string& filename) {
    // open the file for reading
    std::ifstream istr(filename.c_str());
    if (!istr) {
        std::cerr << "ERROR: could not open " << filename << " for reading"
                  << std::endl;
        usage(executable);
    }

    // read in the board dimensions and create an empty board
    // (all outer edges are automatically set to be walls
    int rows, cols;
    istr >> rows >> cols;
    assert(rows > 0 && cols > 0);
    Board answer(rows, cols);

    // read in the other characteristics of the puzzle board
    std::string token;
    while (istr >> token) {
        if (token == "robot") {
            char a;
            int r, c;
            istr >> a >> r >> c;
            answer.placeRobot(Position(r, c), a);
        } else if (token == "vertical_wall") {
            int i;
            double j;
            istr >> i >> j;
            answer.addVerticalWall(i, j);
        } else if (token == "horizontal_wall") {
            double i;
            int j;
            istr >> i >> j;
            answer.addHorizontalWall(i, j);
        } else if (token == "goal") {
            std::string which_robot;
            int r, c;
            istr >> which_robot >> r >> c;
            answer.addGoal(which_robot, Position(r, c));
        } else {
            std::cerr << "ERROR: unknown token in the input file " << token
                      << std::endl;
            exit(0);
        }
    }

    // return the initialized board
    return answer;
}

// ==================================================================================
// ==================================================================================

int main(int argc, char* argv[]) {
    // There must be at least one command line argument, the input puzzle file
    if (argc < 2) {
        usage(argv[0]);
    }

    // By default, the maximum number of moves is unlimited
    int max_moves = -1;

    // By default, output one solution using the minimum number of moves
    bool all_solutions = false;

    // By default, do not visualize the reachability
    char visualize = ' ';  // the space character is not a valid robot!

    // Read in the other command line arguments
    for (int arg = 2; arg < argc; arg++) {
        if (argv[arg] == std::string("-all_solutions")) {
            // find all solutions to the puzzle that use the fewest number of
            // moves
            all_solutions = true;
        } else if (argv[arg] == std::string("-max_moves")) {
            // the next command line arg is an integer, a cap on the  # of moves
            arg++;
            assert(arg < argc);
            max_moves = atoi(argv[arg]);
            assert(max_moves > 0);
        } else if (argv[arg] == std::string("-visualize")) {
            // As a first step towards solving the whole problem, with this
            // option, let's visualize where the specified robot can move
            // and how many steps it takes to get there
            arg++;
            assert(arg < argc);
            std::string tmp = argv[arg];
            assert(tmp.size() == 1);
            visualize = tmp[0];
            assert(isalpha(visualize) && isupper(visualize));
            visualize = toupper(visualize);
        } else {
            std::cout << "unknown command line argument" << argv[arg]
                      << std::endl;
            usage(argv[0]);
        }
    }

    // Load the puzzle board from the input file
    Board board = load(argv[0], argv[1]);

    //
    //
    // YOUR CODE STARTS HERE
    //
    //
    // if (max_moves == -1) {
    //    max_moves = INF;
    //}
    if (visualize == ' ') {
        int cur_moves = 0;
        if(max_moves == -1){
            max_moves = INF;
        }
        do {
            board.set_max_deep(cur_moves);
            board.dfs(0, all_solutions, false);
            cur_moves += 1;
        } while (board.may_have_solution() && cur_moves <= max_moves && !board.find_solution());
    } else {
        board.set_max_deep(max_moves);
        board.dfs(0, all_solutions, visualize != ' ');
    }
    if (visualize != ' ') {
        board.visualize(visualize);
    } else {
        if (board.no_solution()) {
            board.print();
            if (max_moves == -1) {
                std::cout << "no solutions" << std::endl;
            } else {
                std::cout << "no solution with " << max_moves << " or fewer moves" << std::endl;
            }
        } else {
            if (all_solutions) {
                board.pri_all_solution();
            } else {
                board.pri_one_solution();
            }
        }
    }
    // std::cout << "ans = " << ans << std::endl;
    //board.print();
    // board.pri(ans);

    // for now...  an example of how to use the board print function
}

// ================================================================
// ================================================================
