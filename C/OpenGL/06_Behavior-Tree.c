// Adapted from code by Song Ho Ahn (song.ahn@gmail.com)
////////// INIT ////////////////////////////////////////////////////////////////////////////////////

#include "geometry.h"
#include "matrix4x4f.h"
#include <time.h>



////////// PROGRAM SETTINGS ////////////////////////////////////////////////////////////////////////



////////// BEHAVIOR TREES //////////////////////////////////////////////////////////////////////////
#define _BT_STATE_SLOTS 16 // 2024-07-13: At this time, space for state is static

///// BT Enums ////////////////////////////////////////////////////////////

enum BT_Status{
    INVALID, // BT should start here
    RUNNING, // Started but not done
    SUCCESS, // Completed without failure
    FAILURE, // Criteria failed
}; 
typedef enum BT_Status Status;

enum BT_Type{
    LEAF, // --- Ignore children
    SEQUENCE, // Run sequentially to first failure (Has memory!)
    SELECTOR, // Run sequentially to first success
}; 
typedef enum BT_Type BT_Type;


///// BT Structs //////////////////////////////////////////////////////////
// YAGNI: PLEASE KEEP THIS AS LIGHT AS POSSIBLE


typedef struct{
    // Variable data passed between behaviors
    Status  status;
    ulong   tickNum;
    // void*   data;
}BT_Pckt;


typedef struct{
    // Cheapest possible BT struct in C
    BT_Type type; // ---------------------- How should this node run its children?
    char*   name; // ---------------------- Display name of this `Behavior`
    Status  status; // -------------------- Current BT status
    void**  state; // --------------------- Data specific to this `Behavior`
    BT_Pckt (*init  )( void*, BT_Pckt ); // Init   function
    BT_Pckt (*update)( void*, BT_Pckt ); // Update function
    void*   parent; // -------------------- Container
    uint    Nchld; // --------------------- Number of children directly below this node
    void**  children; // ------------------ Children
    uint    index; // --------------------- Currently-running child
}Behavior;


// Default execution packets
BT_Pckt invalid_packet( void* behav, BT_Pckt input ){  BT_Pckt res = {INVALID, input.tickNum};  return res;  }
BT_Pckt running_packet( void* behav, BT_Pckt input ){  BT_Pckt res = {RUNNING, input.tickNum};  return res;  }
BT_Pckt success_packet( void* behav, BT_Pckt input ){  BT_Pckt res = {SUCCESS, input.tickNum};  return res;  }


Behavior* make_action_leaf( char* name_, 
                            BT_Pckt (*initFunc  )( void*, BT_Pckt ), BT_Pckt (*updateFunc  )( void*, BT_Pckt ) ){
    // Create a Behavior that performs an action
    Behavior* rtnBhv = malloc( sizeof( Behavior ) );
    rtnBhv->type     = LEAF; // -------------------------------------- How should this node run its children?
    rtnBhv->name     = name_; // ------------------------------------- Display name of this `Behavior`
    rtnBhv->status   = INVALID; // ----------------------------------- Current BT status
    rtnBhv->state    = malloc( _BT_STATE_SLOTS * sizeof( void* ) ); // Current BT state
    rtnBhv->init     = initFunc; // ---------------------------------- Init   function
    rtnBhv->update   = updateFunc; // -------------------------------- Update function
    rtnBhv->parent   = NULL; // -------------------------------------- Container
    rtnBhv->Nchld    = 0; // ----------------------------------------- Number of children directly below this node
    rtnBhv->children = NULL; // -------------------------------------- Children
    rtnBhv->index    = 0; // ----------------------------------------- Currently-running child
    return rtnBhv;
}

Behavior* make_sequence_container( char* name_, uint N_chldrn ){
    // Create a Behavior that executes a sequence of leaf actions
    Behavior* rtnBhv = malloc( sizeof( Behavior ) );
    rtnBhv->type     = SEQUENCE; // ------------------------------ How should this node run its children?
    rtnBhv->name     = name_; // --------------------------------- Display name of this `Behavior`
    rtnBhv->status   = INVALID; // ------------------------------- Current BT status
    rtnBhv->state    = NULL; // ---------------------------------- Current BT state
    rtnBhv->init     = NULL; // ------------------------------ Init   function
    rtnBhv->update   = NULL; // ---------------------------- Update function
    rtnBhv->parent   = NULL; // ---------------------------------- Container 
    rtnBhv->Nchld    = 0; // ------------------------------------- Number of children directly below this node
    rtnBhv->children = malloc( N_chldrn * sizeof( Behavior ) ); // Children
    rtnBhv->index    = 0; // ------------------------------------- Currently-running child
    return rtnBhv;
}

Behavior* make_selector_container( char* name_, uint N_chldrn ){
    // Create a Behavior that executes a sequence of leaf actions
    Behavior* rtnBhv = malloc( sizeof( Behavior ) );
    rtnBhv->type     = SELECTOR; // ------------------------------- How should this node run its children?
    rtnBhv->name     = name_; // ---------------------------------- Display name of this `Behavior`
    rtnBhv->status   = INVALID; // -------------------------------- Current BT status
    rtnBhv->state    = NULL; // ----------------------------------- Current BT state
    rtnBhv->init     = NULL; // ------------------------------- Init   function
    rtnBhv->update   = NULL; // ----------------------------- Update function
    rtnBhv->parent   = NULL; // ----------------------------------- Container
    rtnBhv->Nchld    = 0; // -------------------------------------- Number of children directly below this node
    rtnBhv->children = malloc( N_chldrn * sizeof( Behavior* ) ); // Children
    rtnBhv->index    = 0; // -------------------------------------- Currently-running child 
    return rtnBhv;
}

void add_child( Behavior* parent, Behavior* child ){
    // Add `child` under `parent`
    parent->children[ parent->Nchld ] = (void*) child;
    child->parent = parent;
    ++(parent->Nchld);
}

// Return the input packet without modification
BT_Pckt pass_packet( void* behav, BT_Pckt input ){  return input;  }

// Defualt init that Discards input && Always runs
BT_Pckt default_init( void* behav, BT_Pckt input ){  return running_packet( behav, input );  }

// Dummy update that Discards input && Always succeeds
BT_Pckt always_succeed( void* behav, BT_Pckt input ){  return success_packet( behav, input );  }


///// BT Methods //////////////////////////////////////////////////////////

Behavior* get_BT_child_i( Behavior* parent, uint i ){
    // Get the `i`th child of this `Behavior`
    if( i < parent->Nchld ){  return (Behavior*) parent->children[i];  }else{  return NULL;  }
}


// BT_Pckt run_init( Behavior* behav, BT_Pckt packet ){
//     // Run init && Set status && Return the init result
//     BT_Pckt res;
//     
//     res = behav->init( behav, packet );
//     // behav->status = res.status;
//     behav->status = RUNNING;
//     return res;
// }





BT_Pckt tick_once( Behavior* behav, BT_Pckt rootPacket ){
    // Advance the BT by one timestep
    Behavior* child_i = NULL; // ----------- Current container child
    BT_Pckt   res_i; // Running execution result

    switch( behav->status ){

        //////////////////////////////////////////

        case INVALID:
            behav->status = RUNNING;
            switch( behav->type ){
                /////////
                case LEAF: 
                    res_i = behav->init( behav, rootPacket );
                    break;
                /////////
                case SEQUENCE:
                case SELECTOR:
                    res_i = running_packet( (void*) behav, rootPacket );
                    behav->index = 0;
                    break;
                /////////
                default:
                    printf( "UNHANDLED BEHAVIOR TYPE!: %i", behav->type );
                    break;
            }
            break;

        //////////////////////////////////////////

        case RUNNING:
            switch( behav->type ){
                /////////
                case LEAF: // Update was already run for leaf, return result
                    res_i /*---*/ = behav->update( behav, rootPacket );
                    behav->status = res_i.status;
                /////////
                case SEQUENCE:
                    printf( "SEQUENCE: Get child at index %u\n", behav->index );
                    child_i = get_BT_child_i( behav, behav->index );
                    tick_once( child_i, res_i );
                    switch( child_i->status ){
                        case SUCCESS:
                            ++(behav->index);
                            printf( "%u >= %u ? %i \n", behav->index, behav->Nchld , behav->index >= behav->Nchld  );
                            if( behav->index >= behav->Nchld ){  behav->status = SUCCESS;  }
                            break;
                        case FAILURE:
                            behav->status = FAILURE;
                            break;
                        default:
                            behav->status = res_i.status;
                            break;
                    }
                /////////
                case SELECTOR:
                    printf( "SEQUENCE: Get child at index %u\n", behav->index );
                    child_i = get_BT_child_i( behav, behav->index );
                    tick_once( child_i, res_i );
                    switch( child_i->status ){
                        case SUCCESS:
                            behav->status = SUCCESS;
                            break;
                        case FAILURE:
                            ++(behav->index);
                            printf( "%u >= %u ? %i \n", behav->index, behav->Nchld , behav->index >= behav->Nchld  );
                            if( behav->index >= behav->Nchld ){  behav->status = FAILURE;  }
                            break;
                        default:
                            behav->status = res_i.status;
                            break;
                    }
                /////////
                default:
                    printf( "UNHANDLED BEHAVIOR TYPE!: %i", behav->type );
                    break;
            }
            break;

        //////////////////////////////////////////
        
        default:
            break;
    }
    
    /// N. Return ///
    return res_i;
}


void reset_tree( Behavior* root ){
    // Set behavior and all children to `INVALID` status, Set containers to initial child
    Behavior* child_i = NULL; // ----------- Current container child
    root->status = INVALID;
    root->index  = 0;
    for( uint i = 0; i < root->Nchld; ++i ){
        child_i = get_BT_child_i( root, i );
        reset_tree( child_i );
    }
}


////////// TIME HELPERS ////////////////////////////////////////////////////////////////////////////

inline long get_epoch_nano( void ){
    // Get nanoseconds since the epoch
    // Author: Ciro Santilli, https://stackoverflow.com/a/36095407
    struct timespec ts;
    timespec_get( &ts, TIME_UTC );
    return (long) ts.tv_sec * 1000000000L + ts.tv_nsec;
}

inline double get_epoch_milli( void ){
    // Get milliseconds since the epoch
    return get_epoch_nano() / ((double) 1e6);
}

// inline void sleep_ms( double pause_ms ){
//     // Main thread will sleep for `pause_ms`
//     // Author: Ciro Santilli, https://stackoverflow.com/q/7684359
//     struct timespec tim, tim2;
//     tim.tv_sec  = (uint) pause_ms / 1000.0;
//     tim.tv_nsec = (uint) (pause_ms - tim.tv_sec) * 1000000;
//     if( nanosleep( &tim, &tim2 ) < 0 ){  printf( "`nanosleep` system call failed \n" );  }
// }

////////// BT RUNNER ///////////////////////////////////////////////////////////////////////////////

typedef struct{
    // Cheapest possible BT manager in C
    Status    status; // -- Current root status
    uint      done; // ---- Has the behavior completed?
    ulong     ts; // ------ Current timestep/tick
    double    period_ms; // Minimum milliseconds between ticks
    double    lastTick; //- Epoch time of last tick, [ms]
    Behavior* root; // ---- BT to run
}BT_Runner;


BT_Runner* setup_BT_w_freq( Behavior* root_, double tickHz ){
    // Get a populated `BT_Runner` struct
    BT_Runner* rtnRnnr = malloc( sizeof( BT_Runner ) );
    rtnRnnr->status    = INVALID;
    rtnRnnr->done      = 0;
    rtnRnnr->ts /*--*/ = 0;
    rtnRnnr->period_ms = (1.0/tickHz)*1000.0;
    rtnRnnr->lastTick  = get_epoch_milli();
    rtnRnnr->root /**/ = root_;
    return rtnRnnr;
}


void reset_BT( BT_Runner* runner ){
    // Reset the runner and associated BT
    runner->status   = INVALID;
    runner->done     = 0;
    runner->ts /*-*/ = 0;
    runner->lastTick = get_epoch_milli();
    reset_tree( runner->root );
}


BT_Pckt run_BT_tick( BT_Runner* runner ){
    // Check time elapsed and conditionally run the BT for one tick
    // NOTE: This functions does NOT sleep the thread, It it up to client code to maintain an update freq
    BT_Pckt rtnPkt = {runner->status, runner->ts}; // --- Prep packet
    double  elp    = get_epoch_milli() - (runner->lastTick); // Calc elapsed time

    printf( "`run_BT_tick`: About to execute tick %lu ...\n", (runner->ts)+1 );

    // If BT period has elapsed since the last call, then execute one tick
    if( (!(runner->done)) && (elp >= (runner->period_ms)) ){
        rtnPkt = tick_once( runner->root, rtnPkt );
        ++(runner->ts);
        runner->status   = rtnPkt.status;
        runner->done     = (uint) ((runner->status == SUCCESS) || (runner->status == FAILURE));
        runner->lastTick = get_epoch_milli();
    }

    // Return status packet
    return rtnPkt;
}


////////// BEHAVIORS ///////////////////////////////////////////////////////////////////////////////

BT_Pckt Countdown_Init( void* behav, BT_Pckt input ){
    ((Behavior*) behav)->state[0] = (void*) input.tickNum;
    return running_packet( behav, input );
}

BT_Pckt Countdown_10( void* behav, BT_Pckt input ){
    // Return `SUCCESS` if 10 ticks have passed, otherwise returning `RUNNING`
    BT_Pckt rtnPkt;
    long    tickBgn = (ulong) ((Behavior*) behav)->state[0];
    // rtnPkt.data    = NULL;
    rtnPkt.tickNum = input.tickNum;
    if( ((long) input.tickNum - tickBgn) < 10 ){  
        rtnPkt.status = RUNNING;  
        printf( "`Countdown_10`: RUNNING at tick %li of 10\n", ((long) input.tickNum - tickBgn) );
    }else{  
        rtnPkt.status = SUCCESS;  
        printf( "Countdown COMPLETE!\n" );
    }
    return rtnPkt;
}


////////// PROGRAM STRUCTS /////////////////////////////////////////////////////////////////////////





////////// PROGRAM STATE ///////////////////////////////////////////////////////////////////////////




////////// MAIN ////////////////////////////////////////////////////////////////////////////////////


int main( int argc, char* argv[] ){
    init_rand();

    double freq_hz   = 60.0;
    long   period_ms = (long) (1.25/freq_hz * 1000.0);
    
    Behavior* seq = make_sequence_container( "Test Sequence", 3 );
    add_child( seq, make_action_leaf( "Counter 1", default_init, Countdown_10 ) );
    add_child( seq, make_action_leaf( "Counter 2", default_init, Countdown_10 ) );
    add_child( seq, make_action_leaf( "Counter 3", default_init, Countdown_10 ) );

    BT_Runner* runner = setup_BT_w_freq( seq, freq_hz );

    while( !(runner->done) ){
        run_BT_tick( runner );
        sleep_ms( period_ms );
    }

    printf( "BT COMPLETED with status %u\n\n", runner->status );
    
    //  Return code
    return 0;
}