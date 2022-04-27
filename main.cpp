#include <string>
#include <cstring>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>
#include <time.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
using namespace std;

// The range of input files that you want to feed as inputs
#define START_INPUT 0
#define END_INPUT 0


// Jump locations
#define JUMP_POS1_I 0
#define JUMP_POS1_J 2
#define JUMP_POS2_I 4
#define JUMP_POS2_J 2
#define JUMP_POS3_I 2
#define JUMP_POS3_J 0
#define JUMP_POS4_I 2
#define JUMP_POS4_J 4

enum Action {U,D,L,R};

/*A_h1 A_h2 IDA_h2 4个函数，分别代表A* 和 迭代A* 及2种启发函数*/
void A_h1(const vector<vector<int> > &start, const vector<vector<int> > &target);
void A_h2(const vector<vector<int> > &start, const vector<vector<int> > &target);
void IDA_h1(const vector<vector<int> > &start, const vector<vector<int> >&target);
void IDA_h2(const vector<vector<int> > &start, const vector<vector<int> > &target);
vector<vector<int>> ReadFile(const char* FileName);
void WritePath(ofstream &fileStream,struct search_state * ptarget);
void WriteFile(ofstream &fileStream,struct search_state * ptarget,double time);
int h1(const vector<vector<int>> &source, const vector<vector<int>> &target);
int h2(const vector<vector<int>> &source, const vector<vector<int>> &target);
int State_Trans(const vector<vector<int> > old_state, vector<vector<int>> &new_state,Action action);
void Print_Vector(const vector<vector<int>> &source_state);
void Print_Path(struct search_state *goal);
void MemFree();
void PQClear();
void CalculateCor(vector<vector<int>> target_state);
int CalManhattan(struct coordinate source_cor,struct coordinate target_cor);

// struct to represent states
struct search_state
{
	int score;
    int depth;
    int action;
    struct search_state* pparent_state;// pointer to parent 
	vector<vector<int>> state;

	// rank by score
	bool operator<(const search_state& s) const
	{
		return this->score > s.score;	// smallest root
	}

	search_state(int s, int d,int a,struct search_state* p,vector<vector<int>> n) :score(s),depth(d),action(a),pparent_state(p),state(n){}
};

// open list, it should be priority queue based on h+g
priority_queue<search_state> open_states;
// store run result for A*/IDA*
struct search_state *ptarget_state=NULL;

// store malloced pointers for future free
vector<struct search_state *> expanded_states;

// struct for planet coordinates
struct coordinate{
    int i;
    int j;
};

// static array for all planet target coordinates
struct coordinate target_cor_array[26];


int main (int argc,char* argv []){
    clock_t start,end;
    double time;
    string fileCount;
    string filePath_start;
    string filePath_target;
    char cfilePath_start[50];
    char cfilePath_target[50];
    vector<vector<int>> source_state;
    vector<vector<int>> target_state;

    ofstream fileStream_Ah1, fileStream_Ah2,fileStream_IDAh1,fileStream_IDAh2;
    fileStream_Ah1.open("../output/output_A_h1.txt", ios::out);
    fileStream_Ah2.open("../output/output_A_h2.txt", ios::out);
    fileStream_IDAh1.open("../output/output_IDA_h1.txt", ios::out);
    fileStream_IDAh2.open("../output/output_IDA_h2.txt", ios::out);


    for(int i=START_INPUT;i<END_INPUT+1;i++){
        // read input file
        fileCount.push_back(i/10+'0');
        fileCount.push_back(i%10+'0');
        filePath_start = "../data/input" + fileCount + ".txt";
        filePath_target = "../data/target" + fileCount + ".txt";

        strcpy(cfilePath_start,filePath_start.c_str());
        strcpy(cfilePath_target,filePath_target.c_str());
        //cout << cfilePath_start << endl;
        //cout << cfilePath_target << endl;
        source_state = ReadFile(cfilePath_start);
        target_state = ReadFile(cfilePath_target);
        // calculate cors for target state
        CalculateCor(target_state);

        //cout<<"Misplaced Stars = "<<h1(source_state,target_state)<<endl;
        cout<<"Mutated Manhattan Distance ="  <<h2(source_state,target_state)<<endl;

        // A_h1
        start = clock();
        //A_h1(source_state,target_state);
        end = clock();
        cout<<"Time = "<<double(end-start)/CLOCKS_PER_SEC<<"s"<<endl;
        time = double(end-start)/CLOCKS_PER_SEC;
        //WriteFile(fileStream_Ah1,ptarget_state,time);
        WriteFile(fileStream_Ah1,NULL,1);
        // free allocated memory
        MemFree();
        // clear pirority queue
        PQClear();
        
        
        

        // A_h2
        
        start = clock();
        //A_h2(source_state,target_state);
        end = clock();
        cout<<"Time = "<<double(end-start)/CLOCKS_PER_SEC<<"s"<<endl;
        time = double(end-start)/CLOCKS_PER_SEC;
        //WriteFile(fileStream_Ah2,ptarget_state,time);
        WriteFile(fileStream_Ah2,NULL,time);
        // free allocated memory
        MemFree();
        PQClear();
        


        // IDA_h1
        start = clock();
       // IDA_h1(source_state,target_state);
        end = clock();
        cout<<"Time = "<<double(end-start)/CLOCKS_PER_SEC<<"s"<<endl;
        time = double(end-start)/CLOCKS_PER_SEC;
        //WriteFile(fileStream_IDAh1,ptarget_state,time);
        WriteFile(fileStream_IDAh1,NULL,time);
        // free allocated memory
        MemFree();
        PQClear();
        



        // IDA_h2
        start = clock();
        //IDA_h2(source_state,target_state);
        end = clock();
        cout<<"Time = "<<double(end-start)/CLOCKS_PER_SEC<<"s"<<endl;
        time = double(end-start)/CLOCKS_PER_SEC;
        //WriteFile(fileStream_IDAh2,ptarget_state,time);
        WriteFile(fileStream_IDAh2,NULL,time);
        // free allocated memory
        MemFree();
        PQClear();
        
        
        fileCount.pop_back();
        fileCount.pop_back();
    }
    fileStream_Ah1.close();
    fileStream_Ah2.close();
    fileStream_IDAh1.close();
    fileStream_IDAh2.close();
    return 0;
}


// clear a priority queue
void PQClear(){
    while(!open_states.empty()){
        open_states.pop();
    }
}


/*read input matrix*/
vector<vector<int>> ReadFile(const char* FileName){
    vector<vector<int>> state{};  
    int row = 5;
    int count = 0;
    string tmp;   
    ifstream fileStream;
    fileStream.open(FileName, ios::in);    
    if (fileStream.fail()) {
        throw logic_error("Read File Failed");
    }else{
        while (getline(fileStream, tmp, '\n')){     
            vector<int> tmpV{};
            istringstream is(tmp);      
            for(int i=0;i<row;i++){
                string str_tmp;
                is>>str_tmp;
                tmpV.push_back(stoi(str_tmp));
            }
            state.push_back(tmpV);
            count++;
        }
        fileStream.close();
    }
    return state;
}


// write helper: write path into file
void WritePath(ofstream &fileStream,struct search_state * ptarget){
    if(ptarget->pparent_state==NULL){
        return;
    }
    WritePath(fileStream,ptarget->pparent_state);
    switch(ptarget->action){
        case L: 
            fileStream << "L";
            break;
        case R:
            fileStream << "R";
            break;
        case U:
            fileStream << "U";
            break;
        case D:
            fileStream << "D";
            break;
    } 
    if(ptarget->score-ptarget->depth==0){
        fileStream << ",";
    }
}

/*write result: operations and */
void WriteFile(ofstream &fileStream,struct search_state * ptarget,double time){
    if (fileStream.fail()) {
        throw logic_error("Write File Failed");
    }
    else if(ptarget == NULL){
        // write "x,x" into the file
        fileStream << "x,x\n";
    }
    else{
        // write path
        WritePath(fileStream,ptarget);
        // write time
        fileStream << time << "\n";
    }
}

// h1: count displaced stars
// can also be used to check if goal state has been reached
int h1(const vector<vector<int>> &source, const vector<vector<int>> &target){
    int count=0;
    // count displacement numbers
    for(int i=0;i<source.size();i++){
        for (int j = 0; j < source[0].size(); ++j) {
            if(source[i][j] != target[i][j]){
                count ++;
            }
        }
    }
    return count;
}

/* h2: mutated mahattan distance 
* calculate min(md,L1,L2,L3,L4) where 
* 1. md is mahattan distance. md(a,b) represents the mahattan distance between a and b
* 2. Lx = md(a,gate x) + md (gate x,b). a is current location and b is target location. gate x is one of the star gate for jumping(4 in total)
* 
* This herustic is admissible because the real cost, which is the number of steps needed, is always smaller than the sum of least moves for all stars to 
* get to the target location. And "the least move" here is not always manhattan distance since there is shorcut via star gate.
*/
int h2(const vector<vector<int>> &source, const vector<vector<int>> &target){
    int md=0;
    int L1,L2,L3,L4;
    int sum = 0;
    struct coordinate cor_source;
    struct coordinate cor_target;
    struct coordinate cor_gate1;
    struct coordinate cor_gate2;
    vector<int> temp_vec{0,0,0,0,0};
    // iterate all planets and get a sum of min(md,L1,L2,L3,L4)
    // coordinates of all planets should be genrated before calling h2 to avoid unnecessary steps

    //TODO: should consider cases where blackhole is located on a star gate (if this case is a legal case)
    for(int i=0;i<source.size();i++){
        for (int j = 0; j < source[0].size(); ++j) {
            if(source[i][j] > 0){
                cor_target = target_cor_array[source[i][j]];
                cor_source.i = i;
                cor_source.j = j;
                md = CalManhattan(cor_source,cor_target);
                cor_gate1.i = JUMP_POS1_I;
                cor_gate1.j = JUMP_POS1_J;
                cor_gate2.i = JUMP_POS2_I;
                cor_gate2.j = JUMP_POS2_J;
                L1 = CalManhattan(cor_source,cor_gate1)+CalManhattan(cor_gate2,cor_target)+1; // current pos to star gate + bound star gate to target pos+1
                L2 = CalManhattan(cor_source,cor_gate2)+CalManhattan(cor_gate1,cor_target)+1;
                cor_gate1.i = JUMP_POS3_I;
                cor_gate1.j = JUMP_POS3_J;
                cor_gate2.i = JUMP_POS4_I;
                cor_gate2.j = JUMP_POS4_J;
                L3 = CalManhattan(cor_source,cor_gate1)+CalManhattan(cor_gate2,cor_target)+1;
                L4 = CalManhattan(cor_source,cor_gate2)+CalManhattan(cor_gate1,cor_target)+1;
                temp_vec[0] =md;temp_vec[1] =L1;temp_vec[2] =L2;temp_vec[3] =L3;temp_vec[4] =L4;
                sum += *min_element(temp_vec.begin(), temp_vec.end());
            }
        }
    }
    return sum;
}

// helper function: calculate mahatan distance 
// use two cor struct to calculate manhattan distance
int CalManhattan(struct coordinate source_cor,struct coordinate target_cor){
    return (abs(source_cor.i-target_cor.i)+abs(source_cor.j-target_cor.j));
}


// A*
void A_h1(const vector<vector<int>> &start, const vector<vector<int> > &target){
    int f;
    struct search_state *pmin_state=NULL;  // stores minimum state in this round of expanding
    struct search_state *pcur_state=NULL;  // stores the state expanded in last round
    int step_count=0;   // stores steps used 
    Action four_actions[4] = {L,R,U,D}; // four legal actions
    //Action four_actions[5] = {D,R,R,R,L};
    //closed_states.push_back(cur_state);
    f = 0+h1(start,target);
    search_state min_state(f,0,L,NULL,start);// stores minimum state in this round of expanding
    search_state temp_state(f,0,L,NULL,start);
    open_states.push(min_state);
    // since not sure whether herustic is consistent, use tree search
    do{
        step_count++;
        // note we always allocate space for herustic-best node since we need these nodes to 
        // form parent-child relationship. In the end, solution can be retrieved by recursion.
        cout << "Iteration round: " << step_count << "\n" <<endl;
        cout << "Allocating Space" << endl;
        pmin_state = (struct search_state*)malloc(sizeof(struct search_state));
        expanded_states.push_back(pmin_state);
        if(!pmin_state){
            cout << "Not enough space" << endl;
        }
        cout << "Space Allocated" << endl;
        cout << "Open list size:" << open_states.size() << endl;
        Print_Vector(open_states.top().state);
        *pmin_state = open_states.top();
        cout << "Assigned" << endl;
        open_states.pop();
        cout << "Poped" << endl;
        // pmin_state ->score = temp_state.score;
        // pmin_state ->depth = temp_state.depth;
        // pmin_state ->action = temp_state.action;
        // pmin_state ->pparent_state = temp_state.pparent_state;
        // test
        //Print_Vector(start);
        cout << " Current State " <<endl;
        //Print_Vector(pmin_state->state);
        // check goal
        if(pmin_state->score - pmin_state->depth == 0){
            cout << "Goal reached" << endl;
            ptarget_state = pmin_state;
            Print_Path(pmin_state);
            return;
        }


        // add herustic-best node's children to open list(expansion) and construct tree 
        // only consider legal actions(edge and black hole)
        for(int k=0;k<4;k++){
            if(State_Trans(pmin_state->state,temp_state.state,four_actions[k])==1){
                    f = pmin_state->depth+1+h1(temp_state.state,target);
                    temp_state.score = f;
                    temp_state.depth = pmin_state->depth+1;
                    temp_state.action = four_actions[k];
                    temp_state.pparent_state = pmin_state;
                    open_states.push(temp_state);
                    cout << "Action: " << four_actions[k] << " Score: " << f  <<" Depth: " << temp_state.depth << "\n" <<endl;
                }
            else{
                cout << four_actions[k] << " is an invalid move for current state\n" << endl;
            }
        }
    }while(open_states.size()!=0);
    ptarget_state = NULL;
}

void A_h2(const vector<vector<int> > &start, const vector<vector<int> > &target){
    int f;
    struct search_state *pmin_state=NULL;  // stores minimum state in this round of expanding
    struct search_state *pcur_state=NULL;  // stores the state expanded in last round
    int step_count=0;   // stores steps used 
    Action four_actions[4] = {L,R,U,D}; // four legal actions
    f = 0+h2(start,target);
    search_state min_state(f,0,L,NULL,start);// stores minimum state in this round of expanding
    search_state temp_state(f,0,L,NULL,start);
    open_states.push(min_state);
    // since not sure whether herustic is consistent, use tree search
    do{
        step_count++;
        // note we always allocate space for herustic-best node since we need these nodes to 
        // form parent-child relationship. In the end, solution can be retrieved by recursion.
        cout << "Iteration round: " << step_count << "\n" <<endl;
        cout << "Allocating Space" << endl;
        pmin_state = (struct search_state*)malloc(sizeof(struct search_state));
        expanded_states.push_back(pmin_state);
        if(!pmin_state){
            cout << "Not enough space" << endl;
        }
        cout << "Space Allocated" << endl;
        cout << "Open list size:" << open_states.size() << endl;
        Print_Vector(open_states.top().state);
        *pmin_state = open_states.top();
        cout << "Assigned" << endl;
        open_states.pop();
        cout << "Poped" << endl;
        // pmin_state ->score = temp_state.score;
        // pmin_state ->depth = temp_state.depth;
        // pmin_state ->action = temp_state.action;
        // pmin_state ->pparent_state = temp_state.pparent_state;
        // test
        //Print_Vector(start);
        cout << " Current State " <<endl;
        //Print_Vector(pmin_state->state);
        // check goal
        if(pmin_state->score - pmin_state->depth == 0){
            cout << "Goal reached" << endl;
            ptarget_state = pmin_state;
            Print_Path(pmin_state);
            return;
        }


        // add herustic-best node's children to open list(expansion) and construct tree 
        // only consider legal actions(edge and black hole)
        for(int k=0;k<4;k++){
            if(State_Trans(pmin_state->state,temp_state.state,four_actions[k])==1){
                    f = pmin_state->depth+1+h2(temp_state.state,target);
                    temp_state.score = f;
                    temp_state.depth = pmin_state->depth+1;
                    temp_state.action = four_actions[k];
                    temp_state.pparent_state = pmin_state;
                    open_states.push(temp_state);
                    cout << "Action: " << four_actions[k] << " Score: " << f  <<" Depth: " << temp_state.depth << "\n" <<endl;
                }
            else{
                cout << four_actions[k] << " is an invalid move for current state\n" << endl;
            }
        }
    }while(open_states.size()!=0);
    ptarget_state = NULL;
}

// IDA*
void IDA_h1(const vector<vector<int> > &start, const vector<vector<int> >&target){
    // 
    int max_depth = 0;
    int stop_depth = 1024;
    int f;
    struct search_state *pmin_state=NULL;  // stores minimum state in this round of expanding
    struct search_state *pcur_state=NULL;  // stores the state expanded in last round
    int step_count=0;   // stores steps used(all deepening rounds) 
    Action four_actions[4] = {L,R,U,D}; // four legal actions
    while(max_depth<stop_depth){
        // outer loop: deepening each round
        // clear priority queue
        PQClear();
        // if there is enough space, MemFree can be done after the IDA* ends.
        MemFree();
        // inner loop: does an A* search with limited depth
        max_depth++; 
        f = 0+h1(start,target);
        search_state min_state(f,0,L,NULL,start);// stores minimum state in this round of expanding
        search_state temp_state(f,0,L,NULL,start);
        open_states.push(min_state);
        do{
            step_count++;
            // note we always allocate space for herustic-best node since we need these nodes to 
            // form parent-child relationship. In the end, solution can be retrieved by recursion.
            cout << "Iteration round: " << step_count << "\n" <<endl;
            cout << "Allocating Space" << endl;
            pmin_state = (struct search_state*)malloc(sizeof(struct search_state));
            expanded_states.push_back(pmin_state);
            if(!pmin_state){
                cout << "Not enough space" << endl;
            }
            cout << "Space Allocated" << endl;
            cout << "Open list size:" << open_states.size() << endl;
            Print_Vector(open_states.top().state);
            *pmin_state = open_states.top();
            cout << "Assigned" << endl;
            open_states.pop();
            cout << "Poped" << endl;
            // test
            //cout << " Current State " <<endl;
            //Print_Vector(pmin_state->state);

            // check goal
            if(pmin_state->score - pmin_state->depth == 0){
                cout << "Goal reached" << endl;
                ptarget_state = pmin_state;
                Print_Path(pmin_state);
                return;
            }
            // add herustic-best node's children to open list(expansion) and construct tree IF it does not exceed current max depth
            // only consider legal actions(edge and black hole)
            if(pmin_state->depth<max_depth){
                for(int k=0;k<4;k++){
                    if(State_Trans(pmin_state->state,temp_state.state,four_actions[k])==1){
                        f = pmin_state->depth+1+h1(temp_state.state,target);
                        temp_state.score = f;
                        temp_state.depth = pmin_state->depth+1;
                        temp_state.action = four_actions[k];
                        temp_state.pparent_state = pmin_state;
                        open_states.push(temp_state);
                        cout << "Action: " << four_actions[k] << " Score: " << f  <<" Depth: " << temp_state.depth << "\n" <<endl;
                        }
                    else{
                        cout << four_actions[k] << " is an invalid move for current state\n" << endl;
                    }
                }
            }
            else{
                // best node's children will exceed max depth, so no expansion for them
                ;
            }
        }while(open_states.size()!=0);

        cout << "Depth: " << max_depth << "Search End" << endl;
    }
    ptarget_state = NULL;
}
void IDA_h2(const vector<vector<int> > &start, const vector<vector<int> > &target){
    // 
    int max_depth = 5;
    int stop_depth = 1024;
    int f;
    struct search_state *pmin_state=NULL;  // stores minimum state in this round of expanding
    struct search_state *pcur_state=NULL;  // stores the state expanded in last round
    int step_count=0;   // stores steps used(all deepening rounds) 
    Action four_actions[4] = {L,R,U,D}; // four legal actions
    while(max_depth<stop_depth){
        // outer loop: deepening each round
        // clear priority queue
        PQClear();
        // if there is enough space, MemFree can be done after the IDA* ends.
        MemFree();
        // inner loop: does an A* search with limited depth
        max_depth++; 
        f = 0+h2(start,target);
        search_state min_state(f,0,L,NULL,start);// stores minimum state in this round of expanding
        search_state temp_state(f,0,L,NULL,start);
        open_states.push(min_state);
        do{
            step_count++;
            // note we always allocate space for herustic-best node since we need these nodes to 
            // form parent-child relationship. In the end, solution can be retrieved by recursion.
            cout << "Iteration round: " << step_count << "\n" <<endl;
            cout << "Allocating Space" << endl;
            pmin_state = (struct search_state*)malloc(sizeof(struct search_state));
            expanded_states.push_back(pmin_state);
            if(!pmin_state){
                cout << "Not enough space" << endl;
            }
            cout << "Space Allocated" << endl;
            cout << "Open list size:" << open_states.size() << endl;
            Print_Vector(open_states.top().state);
            *pmin_state = open_states.top();
            cout << "Assigned" << endl;
            open_states.pop();
            cout << "Poped" << endl;
            // test
            //cout << " Current State " <<endl;
            //Print_Vector(pmin_state->state);

            // check goal
            if(pmin_state->score - pmin_state->depth == 0){
                cout << "Goal reached" << endl;
                ptarget_state = pmin_state;
                Print_Path(pmin_state);
                return;
            }
            // add herustic-best node's children to open list(expansion) and construct tree IF it does not exceed current max depth
            // only consider legal actions(edge and black hole)
            if(pmin_state->depth<max_depth){
                for(int k=0;k<4;k++){
                    if(State_Trans(pmin_state->state,temp_state.state,four_actions[k])==1){
                        f = pmin_state->depth+1+h2(temp_state.state,target);
                        temp_state.score = f;
                        temp_state.depth = pmin_state->depth+1;
                        temp_state.action = four_actions[k];
                        temp_state.pparent_state = pmin_state;
                        open_states.push(temp_state);
                        cout << "Action: " << four_actions[k] << " Score: " << f  <<" Depth: " << temp_state.depth << "\n" <<endl;
                        }
                    else{
                        cout << four_actions[k] << " is an invalid move for current state\n" << endl;
                    }
                }
            }
            else{
                // best node's children will exceed max depth, so no expansion for them
                ;
            }
        }while(open_states.size()!=0);

        cout << "Depth: " << max_depth << "Search End" << endl;
    }
    ptarget_state = NULL;
}

// helper: state transition. returns 1 when state is actually changed, 0 when no change happens
int State_Trans(const vector<vector<int> > old_state, vector<vector<int>> &new_state,Action action){
    new_state = old_state;
    int i_des=-1;int j_des=-1;
    int flag=0;

    // find where the ship is
    int i_pos=-1;int j_pos=-1;
    for(int i=0;i<old_state.size();i++){
        for (int j = 0; j < old_state[0].size(); ++j) {
            if(old_state[i][j] == 0){
                i_pos = i;
                j_pos = j;
            }
        }
    }

    // state transition, note that unavailable place(black hole, edge) should be considered when selecting child node
    if(action!=L && action!=R && action!=U && action!=D){
        cout << "Not a legal action" << endl;
    }
    // we will calculate destination first(consider edge, then consider jump), then confirm whether destination is a black hole. 
    // On the edge
    // these are illegal moves when on the edge
    else if(action==L && (j_pos==0&&i_pos!=2)){;}
    else if(action==R && (j_pos==4&&i_pos!=2)){;}
    else if(action==U && (i_pos==0&&j_pos!=2)){;}
    else if(action==D && (i_pos==4&&j_pos!=2)){;}

    // these are jump cases
    else if(i_pos==JUMP_POS1_I&&j_pos==JUMP_POS1_J&&action==U){
        i_des = JUMP_POS2_I;
        j_des = JUMP_POS2_J;
    }
    else if(i_pos==JUMP_POS2_I&&j_pos==JUMP_POS2_J&&action==D){
        i_des = JUMP_POS1_I;
        j_des = JUMP_POS1_J;
    }
    else if(i_pos==JUMP_POS3_I&&j_pos==JUMP_POS3_J&&action==L){
        i_des = JUMP_POS4_I;
        j_des = JUMP_POS4_J;
    }
    else if(i_pos==JUMP_POS4_I&&j_pos==JUMP_POS4_J&&action==R){
        i_des = JUMP_POS3_I;
        j_des = JUMP_POS3_J;
    }
    // normal case
    else{
        switch(action){
            case L:
                i_des = i_pos; 
                j_des = j_pos-1;
                break;
            case R:
                i_des = i_pos; 
                j_des = j_pos+1;
                break;
            case U:
                i_des = i_pos-1;
                j_des = j_pos; 
                break;
            case D:
                i_des = i_pos+1;
                j_des = j_pos; 
                break;
            default:
                break;
        }
    }

    // i_des, j_des calculated, check whether it is a black hole
    if(i_des<0){
        flag=0;
    }
    else if(old_state[i_des][j_des]<0){
        flag=0;
    }
    else{
        flag = 1;
        //switch des and pos
        //new_state[i_des][j_des] = 0;
        new_state[i_des][j_des] = old_state[i_pos][j_pos];
        new_state[i_pos][j_pos] = old_state[i_des][j_des];
    }
    return flag;
} 


// helper: print the vector
void Print_Vector(const vector<vector<int>> &source_state){
    cout << "-----" << endl;
    for(int i=0;i<source_state.size();i++){
            for (int j = 0; j < source_state[0].size(); ++j) {
                cout<<source_state[i][j]<<"\t";
            }
            cout<<endl;
    }
}

//helper: print the path
void Print_Path(struct search_state *goal){
    if(goal->pparent_state==NULL){
        return;
    }
    Print_Path(goal->pparent_state);
    switch(goal->action){
        case L: 
            cout << "L";
            break;
        case R:
            cout << "R";
            break;
        case U:
            cout << "U";
            break;
        case D:
            cout << "D";
            break;
    } 
    if(goal->score-goal->depth==0){
        cout << endl;
    }
}

// helper: free allocated memory
void MemFree(){
    while(expanded_states.size()!=0){
        free(expanded_states[expanded_states.size()-1]);
        expanded_states.pop_back(); 
        cout << "Cleaning " << expanded_states.size() << " records" << endl;
    }
} 

// helper: calculate coordinates for all planets
void CalculateCor(vector<vector<int>> target_state){
    for(int i=0;i<target_state.size();i++){
            for (int j = 0; j < target_state[0].size(); ++j) {
                if(target_state[i][j]>0){
                    target_cor_array[target_state[i][j]].i = i;
                    target_cor_array[target_state[i][j]].j = j;
                }
            }
    }
    return;
}
