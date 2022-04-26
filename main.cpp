#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>
#include <time.h>
#include <unistd.h>
using namespace std;

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
int h1(const vector<vector<int>> &source, const vector<vector<int>> &target);
int State_Trans(const vector<vector<int> > old_state, vector<vector<int>> &new_state,Action action);
void Print_Vector(const vector<vector<int>> &source_state);
void Print_Path(struct search_state *goal);

// open_states should be priority queue based on h+g
struct search_state
{
	int score;
    int depth;
    int action;
    struct search_state* pparent_state;// pointer to parent 
	vector<vector<int>> state;

	// 重载 < 运算符，用于优先级比较, 以score来排名
	bool operator<(const search_state& s) const
	{
		return this->score > s.score;	// 小顶堆
	}

	search_state(int s, int d,int a,struct search_state* p,vector<vector<int>> n) :score(s),depth(d),action(a),pparent_state(p),state(n){}
};

priority_queue<search_state> open_states;


int main (int argc,char* argv []){
    clock_t start,end;
    vector<vector<int>> source_state = ReadFile("../data/input03.txt");
    vector<vector<int>> target_state = ReadFile("../data/target03.txt");
    //
    cout<<"Misplaced Stars = "<<h1(source_state,target_state)<<endl;


    // test priority queue
    /*
    search_state s1(89,source_state);
	search_state s2(76,target_state);
	search_state s3(100,target_state);

    
	priority_queue<search_state> p5;
	p5.push(s1);
    s1.score = 10;
    s1.state[0][0] = 0;
	p5.push(s2);
	p5.push(s3);
    while (p5.size())
	{
		cout << p5.top().score << endl;
        Print_Vector(p5.top().state);
		p5.pop();
	}
	cout << endl;
    cout << s1.score << endl;
    */
    

    // A_h1
    start = clock();
    A_h1(source_state,target_state);
    end = clock();
    cout<<"Time = "<<double(end-start)/CLOCKS_PER_SEC<<"s"<<endl;
    // A_h2
    return 0;
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
        if(pmin_state==NULL){
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
        Print_Vector(pmin_state->state);
        // check goal
        if(pmin_state->score - pmin_state->depth == 0){
            cout << "Goal reached" << endl;
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
    //}while(flag!=0);
    }while(open_states.size()!=0);

}

void A_h2(const vector<vector<int> > &start, const vector<vector<int> > &target){}

// IDA*
void IDA_h1(const vector<vector<int> > &start, const vector<vector<int> >&target){}
void IDA_h2(const vector<vector<int> > &start, const vector<vector<int> > &target){}

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
