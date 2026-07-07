/*
 * TABU SEARCH
 * CLASSICAL VEHICLE ROUTING PROBLEM
 * Group F-09
 * TALAL AHMED 19I-0727 CS-F
 * HAJIRA UZAIR 19I-0737 CS-F
 * HYDER ALI MEMON 19I-0703 CS-F
 * KASHIF NIAZI 19I-0640 CS-F
 * */

#include <iostream>
#include <ctime>
#include <vector>
#include <iterator>
#include <cstdlib>
#include <cmath>
#include <chrono>
using namespace std::chrono;
using namespace std;

int oldSolutionIterator = 0;

struct Node {
    // id for each customer, it's x & y axis, demand of order
    // length is total number of customers
    // isRouted is check whether customer order is routed on some vehicle or not
    int id, x, y, demand;
    static int length;
    bool isRouted;

    Node()
    {
        id = -1;
        x = y = demand = 0;
        isRouted = false;
    }

    Node(int dx, int dy) : x(dx), y(dy)
    {
        demand = 0;
        isRouted = false;
        id = 0;
    }

    Node(int cid, int cx, int cy, int dem) : id(cid), x(cx), y(cy), demand(dem)
    {
        isRouted = false;
    }

    void initialize(int cid, int cx, int cy, int dem)
    {
//        cout << "Nodes[" << cid << "] = new Node(" << cid << "," << cx << "," << cy << "," << dem << ");"<<endl;
        id = cid;
        x = cx;
        y = cy;
        demand = dem;
        isRouted = false;
        length++;
    }
};

struct Vehicle {
    // id of each vehicle, capacity it can load, current load in it and it's location
    // length is total number of vehicles
    // route array is path for each vehicle
    int id, capacity, load, location;
    static int length;
    vector<Node> route;

    Vehicle(int vid, int cap) : id(vid), capacity(cap)
    {
        load = location = 0; // in dep
    }

    Vehicle()
    {
        id = capacity = load = location = 0;
    }

    void initialize(int vid, int cap)
    {
        id = vid;
        capacity = cap;
        load = location = 0; // in dep
    }

    void addCustomer(Node customer)
    {
        route.push_back(customer);
        load += customer.demand;
        location = customer.id;
    }

    bool checkCapacity(int dem)
    {
        return (load + dem <= capacity);
    }
};

struct Solution
{
    // no of vehicles and customers in our problem, cost is best solution, vehicles is array of delivery trucks
    int no_of_vehicles, no_of_customers;
    double cost;
    Vehicle* vehicles;
    vector<double> old_solutions;

    // ===================================== TABU SEARCH REQUIREMENTS ========================
    // vehicles array rearranged for tabu search best solution, best cost after applying tabu search
    Vehicle* vehicles_best_solution;
    double best_solution_cost;

    Solution(int cust, int vec, int vcap) : no_of_vehicles(vec), no_of_customers(cust), cost(0)
    {
        vehicles = new Vehicle[no_of_vehicles];
        vehicles_best_solution = new Vehicle[no_of_vehicles];
        for(int i = 0; i < no_of_vehicles; i++)
        {
            vehicles[i].initialize(i+1, vcap);
            vehicles_best_solution[i].initialize(i+1, vcap);
        }
        best_solution_cost = 0;
    }

    // This function returns true if any of the customer's order is left unassigned to any vehicle
    bool anyUnassignedCustomer(Node* customers)
    {
        for(int i = 1; i <= Node::length; i++)
        {
            if(!customers[i].isRouted)
            {
                return true;
            }
        }
        return false;
    }

    void greedySolution(Node* customers, double** cost_mat)
    {
        double customer_cost, end_cost;
        int vid = 0;
        // loop until every customer order is assigned to vehicle
        while(anyUnassignedCustomer(customers))
        {
            int cid = 0;
            Node temp_customer;
            double min = 999999;
            // for any vehicle if it's route is empty assign first customer (that is depot) to it
            if(vehicles[vid].route.empty())
            {
                vehicles[vid].addCustomer(customers[0]);
            }
            // loop for each customer and if customer is not routed on any vehicle before
            // check for the capacity of the truck left and customer's demand if
            // everything is clear then set current customer's cost (distance) from
            // cost matrix. If current customer's cost is less than the current
            // minimum set minimum to the current customer's cost, grab it's index
            // and also the customer at this index.
            for (int i = 1; i <= no_of_customers; i++) {
                if (!customers[i].isRouted) {
                    if (vehicles[vid].checkCapacity(customers[i].demand)) {
                        customer_cost = cost_mat[vehicles[vid].location][i];
                        if (min > customer_cost) {
                            min = customer_cost;
                            cid = i;
                            temp_customer = customers[i];
                        }
                    }
                }
            }
            // if can't find any minimum cost customer
            if (temp_customer.id == -1)
            {
                if(vid+1 < Vehicle::length)
                {
                    // if vehicle is not in depot assign last customer (that is depot) to it's path
                    if(vehicles[vid].location != 0)
                    {
                        end_cost = cost_mat[vehicles[vid].location][0];
                        vehicles[vid].addCustomer(customers[0]);
                        cost += end_cost;
                    }
                    vid += 1;
                }
                else
                {
                    cout << "Capacity doesn't exists \n";
                    exit(0);
                }
            }
            else
            {
                // if found minimum cost customer add it to vehicles path and updated it's status. Add total cost
                vehicles[vid].addCustomer(temp_customer);
                customers[cid].isRouted = true;
                cost += min;
            }
        }
        // if vehicle is not in depot assign last customer (that is depot) to it's path
        end_cost = cost_mat[vehicles[vid].location][0];
        vehicles[vid].addCustomer(customers[0]);
        cost += end_cost;
    }

    // Operators from the first group move one or
    // more customers from one position in the route to another position in the same route and are
    // called Intra Route operators
    void intraRouteLocalSearch(Node* customers, double** cost_mat)
    {
        vector<Node> route; // vehicle route
        double neighborhood_cost, bestNcost; // cost's after swaping customers
        int swap_a = -1, swap_b = -1, swap_route = -1;
        int iterations = 100000; // max iterations allowed for intra route operation
        int i = 0; // current iteration
        bool terminate = false;
        // loop until stopping criteria is achieved
        while(!terminate)
        {
            i++;
            bestNcost = INT8_MAX;
            // loop for each vehicle in company
            for(int vid = 0; vid < Vehicle::length; vid++)
            {
                route = vehicles[vid].route;
                int route_length = route.size();
                // loop from first customer to the last
                for(int j = 1; j < route_length - 1; j++)
                {
                    // loop from depot to the last customer
                    for(int k = 0; k < route_length - 1; k++)
                    {
                        // ensure both customer's are not same
                        if((j != k) && (j-1 != k)) {
                            double subtracted_cost1 = cost_mat[route[j-1].id][route[j].id];
                            double subtracted_cost2 = cost_mat[route[j].id][route[j+1].id];
                            double subtracted_cost3 = cost_mat[route[k].id][route[k+1].id];

                            double added_cost1 = cost_mat[route[j-1].id][route[j+1].id];
                            double added_cost2 = cost_mat[route[k].id][route[j].id];
                            double added_cost3 = cost_mat[route[j].id][route[k+1].id];

                            neighborhood_cost = added_cost1 + added_cost2 + added_cost3 -
                                                subtracted_cost1 - subtracted_cost2 - subtracted_cost3;
                            // getting cost after swapping customers j & k
                            // if there is some improvement in cost change bestNcost,
                            // swap indices and swap route index to current vehicle
                            if(neighborhood_cost < bestNcost)
                            {
                                bestNcost = neighborhood_cost;
                                swap_a = j;
                                swap_b = k;
                                swap_route = vid;
                            }
                        }
                    }
                }
            }
            // if improvement swap both customers for vehicle and update it's route
            if (bestNcost < 0)
            {
                Node Swap_customer = vehicles[swap_route].route[swap_a];

                vehicles[swap_route].route.erase(vehicles[swap_route].route.begin() + swap_a);
                if(swap_a < swap_b)
                {
                    vehicles[swap_route].route.insert(vehicles[swap_route].route.begin()+swap_b, Swap_customer);
                }
                else
                {
                    vehicles[swap_route].route.insert(vehicles[swap_route].route.begin()+(swap_b+1), Swap_customer);
                }
                old_solutions.push_back(cost);
                cost += bestNcost;
            }
            else
            {
                // if no improvement terminate
                terminate = true;
            }

            if(i >= iterations){
                // another stopping criteria
                terminate = true;
            }
        }
        old_solutions.push_back(cost);
        cout << "IntraRoute Old Solutions :: \n";
        for(double old_solution : old_solutions)
        {
            oldSolutionIterator++;
            if(old_solution >= 10 && old_solution <= 999999){
                cout << old_solution << endl;
            }
        }

    }

    // The Tabu search (TS) is a well-known meta-heuristic
    // possibly used for such improvement, which has also been successfully applied to solve
    // many other combinatorial optimization problems. In solving VRPs, given any initial
    // route(s), there can be many possible moves such as the 2-opt operation, which replaces
    // any two links in a route with two different links to reduce the operational cost, to generate
    // other possible route(s). The Tabu search works by firstly performing a neighborhood search
    // on all possible moves, executing only non-Tabu moves which reduce the total operational
    // cost, and ‘memorizing’ those recently performed moves with a Tabu list, usually of fixed
    // length, to avoid cycling. In this way, TS promotes a diversified search from the current
    // solution by continuously updating a short-term memory-like Tabu list until the
    // predetermined stopping criterion is reached
    void tabuSearch(int tabu_size, double** cost_mat)
    {
        int _terminate = 0;
        vector<Node> routeFrom, routeTo;
        int movingCustomerDemand = 0;
        int vidFrom, vidTo;
        double bestNCost, NeighborCost;
        int swap_a = -1, swap_b = -1, swap_from = -1, swap_to = -1;
        int iterations = 100; // max iterations allowed
        int i = 0;
        int dim_customer = no_of_customers+1;
        // Tabu matrix is concept of tabu list by storing previous best solurions for each customer
        int** tabu_matrix = new int*[dim_customer+1];
        for(int m = 0; m < dim_customer+1; m++){
            tabu_matrix[m] = new int[dim_customer+1];
            for(int n = 0; n < dim_customer+1; n++){
                tabu_matrix[m][n] = 0;
            }
        }
        bool terminate = false;
        best_solution_cost = cost;
        while(!terminate)
        {
            i++;
            bestNCost = 999999;
            // iterate through each vehicle in company
            for(vidFrom = 0; vidFrom < Vehicle::length; vidFrom++)
            {
                routeFrom = vehicles[vidFrom].route;
                int route_length_from = routeFrom.size();
                // for each customer in Vehicle's A route iterate through each vehicle
                for(int j = 1; j < route_length_from - 1; j++)
                {
                    // for each customer in Vehicle's A route iterate through each vehicle
                    for(vidTo = 0; vidTo < Vehicle::length; vidTo++)
                    {
                        routeTo = vehicles[vidTo].route;
                        int route_length_to = routeTo.size();
                        // iterate through vehicle's B route
                        for(int k = 0; k < route_length_to - 1; k++)
                        {
                            movingCustomerDemand = routeFrom[j].demand;
                            // check that both vehicles aren't same and vehicle has capacity to fulfil customer's demand
                            if((vidFrom == vidTo) || (vehicles[vidTo].checkCapacity(movingCustomerDemand)))
                            {
                                // Check that both customers aren't same
                                if(!((vidFrom == vidTo) && ((j == k) || (k == j - 1))))
                                {
                                    double subtracted_cost1 = cost_mat[routeFrom[j-1].id][routeFrom[j].id];
                                    double subtracted_cost2 = cost_mat[routeFrom[j].id][routeFrom[j+1].id];
                                    double subtracted_cost3 = cost_mat[routeTo[k].id][routeTo[k+1].id];

                                    double added_cost1 = cost_mat[routeFrom[j-1].id][routeFrom[j+1].id];
                                    double added_cost2 = cost_mat[routeTo[k].id][routeFrom[j].id];
                                    double added_cost3 = cost_mat[routeFrom[j].id][routeTo[k+1].id];
                                    // If solution is already in tabu list break
                                    if((tabu_matrix[routeFrom[j-1].id][routeFrom[j+1].id] != 0) ||
                                       (tabu_matrix[routeFrom[k].id][routeFrom[j].id] != 0) ||
                                       (tabu_matrix[routeFrom[j].id][routeFrom[k+1].id] != 0))
                                    {
                                        break;
                                    }

                                    NeighborCost = added_cost1 + added_cost2 + added_cost3 -
                                                   subtracted_cost1 - subtracted_cost2 - subtracted_cost3;
                                    // getting cost after swapping customer j in Vehicle's A route &
                                    // customer k in Vehicle's B route if there is some improvement
                                    // in cost change bestNcost, swap indices and swap route index
                                    // of both vehicles
                                    if(NeighborCost < bestNCost)
                                    {
                                        bestNCost = NeighborCost;
                                        swap_a = j;
                                        swap_b = k;
                                        swap_from = vidFrom;
                                        swap_to = vidTo;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            for(int j = 0; j < dim_customer+1; j++)
            {
                for(int k = 0; k < dim_customer+1; k++)
                {
                    if(tabu_matrix[j][k] > 0)
                    {
                        tabu_matrix[j][k]--;
                    }
                }
            }


            routeFrom = vehicles[swap_from].route;
            routeTo = vehicles[swap_to].route;

            Node Swap_custoemr = routeFrom[swap_a];
            int previd = routeFrom[swap_a-1].id;
            int nextid = routeFrom[swap_a+1].id;
            int fid = routeFrom[swap_b].id;
            int gid = routeFrom[swap_b+1].id;

            vehicles[swap_from].route.erase(vehicles[swap_from].route.begin() + swap_a);

            if(swap_from == swap_to)
            {
                if(swap_a < swap_b)
                {
                    vehicles[swap_to].route.insert(vehicles[swap_to].route.begin()+swap_b, Swap_custoemr);
                }
                else
                {
                    vehicles[swap_to].route.insert(vehicles[swap_to].route.begin()+(swap_b+1), Swap_custoemr);
                }
            } else {
                vehicles[swap_to].route.insert(vehicles[swap_to].route.begin()+(swap_b+1), Swap_custoemr);
            }

            vehicles[swap_from].load -= movingCustomerDemand;
            vehicles[swap_to].load += movingCustomerDemand;
            vector<double>::iterator ptr;
            ptr = old_solutions.end();
            if(*ptr < cost){
                _terminate++;
            }
            old_solutions.push_back(cost);
            cost += bestNCost;

            // if after tabu-search there is some improvement save best minimum cost
            if(cost < best_solution_cost)
            {
                SaveBestSolution();
            }

            if(i >= iterations || _terminate == 30)
            {
                // another stopping criteria
                terminate = true;
            }
        }
        // update cost and vehicles array
        vehicles = vehicles_best_solution;

        cost = best_solution_cost;

        int _current = 0;
        cout << "TabuSearch Old Solutions :: \n";
        for(double old_solution : old_solutions)
        {
            if(_current >= oldSolutionIterator){
                if(old_solution < 999999){
                    cout << old_solution << endl;
                }
            }
            _current++;
        }
    }

    // Save tabu search best solution
    void SaveBestSolution()
    {
        best_solution_cost = cost;
        for(int i = 0; i < no_of_vehicles; i++)
        {
            vehicles_best_solution[i].route.clear();
            if(!vehicles[i].route.empty())
            {
                int route_size = vehicles[i].route.size();
                for(int j = 0; j < route_size; j++)
                {
                    Node temp = vehicles[i].route[j];
                    vehicles_best_solution[i].route.push_back(temp);
                }
            }
        }
    }
    void print()
    {
        for(int i = 0; i < no_of_vehicles; i++)
        {
            if(!vehicles[i].route.empty())
            {
                cout << "Vehicle " << i << " : ";
                int rsize = vehicles[i].route.size();
                for(int j = 0; j < rsize; j++){
                    if(j == rsize-1)
                    {
                        cout << vehicles[i].route[j].id;
                    }
                    else
                    {
                        cout << vehicles[i].route[j].id << "->";
                    }
                }
                cout << endl;
            }
        }
        cout << cost << endl;
    }

};

int Node::length = 0;
int Vehicle::length = 0;

int main()
{
    srand(time(nullptr));
    int no_of_customers = 500;
    int vehicles = 2500;
    int vehicles_capacity = 50;
    Vehicle::length = vehicles;
    int dep_x = 50, dep_y = 50;

    int TABU = 100;
    cout << "Total Number of Customers :: " << no_of_customers << endl;
    cout << "Capacity of Vehicles :: " << vehicles_capacity << endl;
    cout << "Vehicles Avaliable :: " << vehicles << endl;
    Node customers[no_of_customers+1];
    Node dep(dep_x, dep_y);
    customers[0] = dep;

    for(int i = 1; i <= no_of_customers; i++){
        int x = rand()%100, y = rand()%100, dem = 4+rand()%7;
        customers[i].initialize(i,
                                x,
                                y,
                                dem);
    }
    cout << endl;
    double** distance_matrix = new double*[no_of_customers+1];
    for(int i = 0; i < no_of_customers+1; i++){
        distance_matrix[i] = new double[no_of_customers+1];
        for(int j = 0; j < no_of_customers+1; j++){
            distance_matrix[i][j] = 0;
        }
    }
    double x_coord, y_coord, dist;
    for(int i = 0; i <= no_of_customers; i++)
    {
        for(int j = i+1; j <= no_of_customers; j++)
        {
            x_coord = customers[i].x - customers[j].x;
            y_coord = customers[i].y - customers[j].y;
            dist = round(sqrt(pow(x_coord, 2) + pow(y_coord, 2)));
            distance_matrix[i][j] = dist;
            distance_matrix[j][i] = dist;
        }
    }

//    for(int i = 0; i <= no_of_customers; i++)
//    {
//        for(int j = 0; j <= no_of_customers; j++)
//        {
//            cout << distance_matrix[i][j] << ", ";
//        }
//        cout << endl;
//    }
    auto start = high_resolution_clock::now();
    Solution* s = new Solution(no_of_customers, vehicles, vehicles_capacity);
    s->greedySolution(customers, distance_matrix);
    s->print();
    s->intraRouteLocalSearch(customers, distance_matrix);
    s->print();
    s->tabuSearch(TABU, distance_matrix);
    s->print();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop - start);
    cout << "RUN TIME :: " << duration.count() << endl;
}
