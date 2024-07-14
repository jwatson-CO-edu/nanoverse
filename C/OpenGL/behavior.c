////////// INIT ////////////////////////////////////////////////////////////////////////////////////
#include "toolbox.h"



////////// BEHAVIOR TREES //////////////////////////////////////////////////////////////////////////

// Default execution packets
BT_Pckt invalid_packet( void* behav, BT_Pckt input ){  BT_Pckt res = {INVALID, input.tickNum};  return res;  }
BT_Pckt running_packet( void* behav, BT_Pckt input ){  BT_Pckt res = {RUNNING, input.tickNum};  return res;  }
BT_Pckt success_packet( void* behav, BT_Pckt input ){  BT_Pckt res = {SUCCESS, input.tickNum};  return res;  }
BT_Pckt failure_packet( void* behav, BT_Pckt input ){  BT_Pckt res = {FAILURE, input.tickNum};  return res;  }


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
    rtnBhv->init     = NULL; // ---------------------------------- Init   function
    rtnBhv->update   = NULL; // ---------------------------------- Update function
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
    rtnBhv->init     = NULL; // ----------------------------------- Init   function
    rtnBhv->update   = NULL; // ----------------------------------- Update function
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


BT_Pckt tick_once( Behavior* behav, BT_Pckt rootPacket ){
    // Advance the BT by one timestep
    Behavior* child_i = NULL; // ----------- Current container child
    BT_Pckt   res_i; // Running execution result

    printf( "About to run Node %s at address %p ...\n", behav->name, behav );

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
                    break;
                /////////
                case SEQUENCE:
                    printf( "SEQUENCE %s: Get child at index %u\n", behav->name, behav->index );
                    child_i = get_BT_child_i( behav, behav->index );
                    if( child_i ){
                        tick_once( child_i, rootPacket );
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
                        res_i.status  = behav->status;
                        res_i.tickNum = rootPacket.tickNum;
                    }else{
                        printf( "ERROR: NO child of %s at index %u!\n", behav->name, behav->index );
                        res_i = failure_packet( behav, rootPacket );
                    }
                    break;
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
                    break;
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


////////// BT RUNNER ///////////////////////////////////////////////////////////////////////////////

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

    // If BT period has elapsed since the last call, then execute one tick
    if( (!(runner->done)) && (elp >= (runner->period_ms)) ){
        printf( "`run_BT_tick`: About to execute tick %lu ...\n", (runner->ts)+1 );
        rtnPkt = tick_once( runner->root, rtnPkt );
        ++(runner->ts);
        runner->status   = rtnPkt.status;
        runner->done     = (uint) ((runner->status == SUCCESS) || (runner->status == FAILURE));
        runner->lastTick = get_epoch_milli();
    }

    // Return status packet
    return rtnPkt;
}