///// Behavior Trees //////////////////////////////////////////////////////
enum BT_Status{
  INVALID,
  RUNNING,
  SUCCESS,
  FAILURE,
}; 
typedef enum BT_Status Status;

typedef struct{
    Status status;
    Status (*update)( void );
    void*  parent;
    void** children;
}Behavior;