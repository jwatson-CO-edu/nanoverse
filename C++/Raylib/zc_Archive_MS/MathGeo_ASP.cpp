/***********  
MathGeo_ASP.cpp
James Watson , 2018 January
Basic Math and 2D / 3D Geometry Utilities

Template Version: 2017-09-23
***********/

#include <MathGeo_ASP.h>

// === Geometry Classes ===
	
// == class RAPID_Geo ==
	
RAPID_Geo::RAPID_Geo(){
	Rmodel = new RAPID_model;
	for( size_t i = 0 ; i < 3 ; i++ ){
		T[i] = 0.0; 
		for( size_t j = 0 ; j < 3 ; j++ ){
			R[i][j] = 0.0;
		}
	}
}

RAPID_Geo::~RAPID_Geo(){
	delete   Rmodel;
}

void RAPID_Geo::load_RAPID_model( const Eigen::MatrixXd& V , const Eigen::MatrixXi& F ){
	// Load all of the points into 'rapidModel' , Which stores points in the C style
	// NOTE: This function does not assume a "sparse" or "dense" style , Either will work
	int counter = 0; // Triangle ID counter , This will follow F
	int len     = F.rows();
	double p0[3], p1[3], p2[3]; // R3 points
	numTris = 0;
	// 0. Clear the old model of data
	if( Rmodel ){  delete Rmodel;  Rmodel = new RAPID_model;  }
	// 1. Prepare the model
	Rmodel->BeginModel();
	// 2. For each f_i
	for( int i = 0 ; i < len ; i++ ){
		// 3. For each dimension of v_{0,1,2}
		for( int j = 0 ; j < 3 ; j++ ){
			// 4. Load the point into the array
			p0[j] = V( F(i,0) , j );
			p1[j] = V( F(i,1) , j );
			p2[j] = V( F(i,2) , j );
		}
		// 5. Load the facet, and give it a unique number (unique to the model, does not have to be unique globally)
		Rmodel->AddTri( p0 , p1 , p2 , counter );
		counter++; // This will follow F
	}
	numTris = counter;
	// 6. End model input
	Rmodel->EndModel();
}

void RAPID_Geo::load_RAPID_model( const TriMeshVFN_ASP& pMesh ){
	// Load all of the points into 'rapidModel' , Which stores points in the C style
	// NOTE: This function does not assume a "sparse" or "dense" style , Either will work
	load_RAPID_model( pMesh.V , pMesh.F );
}

void RAPID_Geo::load_RAPID_transform( Pose_ASP partPose ){
	// Extract the pose information 'partPose' and load it into 'T' and 'R' , as RAPID expects
	// NOTE: Collision detection *REQUIRES* that the 'partPose' homogeneous transformation matrix has been properly populated
	for( size_t i = 0 ; i < 3 ; i++ ){
		T[i] = partPose.homogMatx( i , 3 ); // Transfer the Pose_ASP translation to the RAPID translation
		for( size_t j = 0 ; j < 3 ; j++ ){
			R[i][j] = partPose.homogMatx( i , j );
		}
	}
}

bool RAPID_Geo::collides_with( RAPID_Geo& other ){
	// Determine collision with another 'RAPID_Geo' 
	RAPID_Collide( R       , T       , Rmodel       , 
	               other.R , other.T , other.Rmodel , RAPID_FIRST_CONTACT ); // Shortcut on first triangle collision
	if( RAPID_num_contacts > 0 ){  return true;  }else{  return false;  }
}

std::set<int> RAPID_Geo::collision_F( RAPID_Geo& other ){
	// Determine all triangles in collision with another 'RAPID_Geo' 
	
	bool SHOWDEBUG = false; // if( SHOWDEBUG ){  cout << "" << endl;  }
	
	std::set<int> rtnSet;
	RAPID_Collide( R       , T       , Rmodel       , 
	               other.R , other.T , other.Rmodel , RAPID_ALL_CONTACTS ); // Exhaustive search of all contacts
	if( SHOWDEBUG ){  cout << "There are " << RAPID_num_contacts << " contacts to log" << endl;  }
	for( int i = 0 ; i < RAPID_num_contacts ; i++ ){  
		rtnSet.insert( RAPID_contact[i].id1 );  
	}
	return rtnSet;
}

// __ End RAPID_Geo __


// == class CollisionVFN_ASP ==

// ~ Con/Destructors ~
CollisionVFN_ASP::CollisionVFN_ASP( const TriMeshVFN_ASP& pMesh ){  init_for_collision( pMesh );  }

// ~ Methods ~

void CollisionVFN_ASP::init_for_collision( const TriMeshVFN_ASP& pMesh ){
	// NOTE: This function assumes that the mesh arrived properly posed
	mesh = copy_TriMeshVFN_ASP( pMesh );
	aabb = AABB( pMesh );
	collsn_geo.load_RAPID_model( pMesh );
	Pose_ASP zeroT = pose_from_position_orient( origin_vec() , no_turn_quat() ); // No transform is applied
	collsn_geo.load_RAPID_transform( zeroT );
}

// __ End CollisionVFN_ASP __

// ___ End Classes ___


// === General Mathematics ===

size_t triangular_num( size_t N ){  return N * ( N+1 ) / 2;  } // Return the Nth triangular number

double selected_average_vec( std::vector<double>& vec , std::vector<size_t>& selection ){ 
	// Return the average of the selected points , R1
	size_t len   = selection.size();
	double total = 0.0;
	for( size_t i = 0 ; i < len ; i++ ){  total += vec[ selection[ i ] ];  }
	return total / (double) len;
}

double vec_L2_norm( std::vector<double>& vec ){
	// Return the Euclidean Norm of a standard vector as though it were an n-dim vector
	double sqrdSum = 0.0;
	size_t len     = vec.size();
	for( size_t i = 0 ; i < len ; i++ ){  sqrdSum += pow( vec[i] , 2 );  }
	return sqrt( sqrdSum );
}

double vec_L2_diff( std::vector<double>& v1 , std::vector<double>& v2 ){ 
	// Return the Euclidean Norm of the difference between standard vectors
	size_t len1 = v1.size();
	std::vector<double> resultant;
	if( len1 == v2.size() ){
		for( size_t i = 0 ; i < len1 ; i++ ){  resultant.push_back( v1[i] - v2[i] );  }
		return vec_L2_norm( resultant );
	}else{  return nan("");  }
}

std::vector<double> perturb_vec( std::vector<double>& vec , double oneSideRange ){
	// Perturb each element of 'vec' by up to +/- 'oneSideRange'
	std::vector<double> rtnVec;
	size_t len = vec.size();
	for( size_t i = 0 ; i < len ; i++ ){  rtnVec.push_back( vec[i] + randrange( -oneSideRange , oneSideRange ) );  }
	return rtnVec;
}

// ___ End General ___


// === Geometry Functions ===

// == Struct Helpers ==

Pose_ASP pose_from_position_orient( const Eigen::Vector3d& position , const Eigen::Quaterniond& orientation ){
	Pose_ASP rtnStruct;
	rtnStruct.position    = position;
	rtnStruct.orientation = orientation.normalized();
	rtnStruct.homogMatx   = Eigen::MatrixXd::Zero( 4 , 4 );
	Eigen::MatrixXd R = rtnStruct.orientation.toRotationMatrix();
	rtnStruct.homogMatx.block<3,3>(0,0) = R;
	for( size_t i = 0 ; i < 3 ; i++ ){  rtnStruct.homogMatx( i , 3 ) = position( i );  }
	rtnStruct.homogMatx( 3 , 3 ) = 0.0;
	return rtnStruct;
}

Pose_ASP pose_from_pXYZ_oWXYZ( double pX , double pY , double pZ , double oW , double oX , double oY , double oZ ){
	Pose_ASP rtnStruct;
	rtnStruct.position    = Eigen::Vector3d( pX , pY , pZ );
	rtnStruct.orientation = Eigen::Quaterniond( oW , oX , oY , oZ ).normalized();
	return rtnStruct;
}

Pose_ASP pose_from_origin_bases( const Eigen::Vector3d& origin ,
								 const Eigen::Vector3d& xBasis , const Eigen::Vector3d& yBasis , const Eigen::Vector3d& zBasis ){
	return pose_from_position_orient( origin , 
									  basis_vecs_to_quat( xBasis , yBasis , zBasis ) );
}

Eigen::Quaterniond operator/( const Eigen::Quaterniond& left_target , const Eigen::Quaterniond& rght_base ){
	// Return the Quaternion that will turn 'rght_base' onto 'left_target'
	if( eq( left_target , rght_base ) ){
		return no_turn_quat();
	}else{
		return rght_base.inverse() * left_target;
	}
}

double mag_rad( Eigen::Quaterniond turn ){  return 2 * acos( turn.w() );  }

double angle_to_from( const Eigen::Quaterniond left_target , const Eigen::Quaterniond rght_base ){  
	// Return the angle of the shortest turn to 'left_target' from 'rght_base'
	return mag_rad( left_target / rght_base );
}

Pose_ASP operator*( const Pose_ASP& left_op , const Pose_ASP& rght_op ){
	// Return the lab frame pose of 'rght_op' as though it were a expressed in the frame 'left_op'
	Eigen::Vector3d pos = ( left_op.orientation * rght_op.position ) + left_op.position;
	Eigen::Quaterniond orient = left_op.orientation * rght_op.orientation;
	orient.normalize();
	return pose_from_position_orient( pos , orient );
}

Pose_ASP operator/( const Pose_ASP& left_target , const Pose_ASP& rght_base ){
	// Return the Pose that will move 'rght_base' onto 'left_target'
	Eigen::Vector3d pos = rght_base.orientation.inverse() * ( left_target.position - rght_base.position );
	Eigen::Quaterniond orient = left_target.orientation / rght_base.orientation;
	return pose_from_position_orient( pos , orient );
}

Pose_ASP origin_pose(){  return Pose_ASP{ Eigen::Vector3d( 0.0 , 0.0 , 0.0 ) , Eigen::Quaterniond( 1.0 , 0.0 , 0.0 , 0.0 ) };  }

Pose_ASP copy_pose( const Pose_ASP& original ){  
	return Pose_ASP{ 
		Eigen::Vector3d( 
			original.position(0) ,  
			original.position(1) ,
			original.position(2) 
		) , 
		Eigen::Quaterniond( 
			original.orientation.w() ,  
			original.orientation.x() , 
			original.orientation.y() , 
			original.orientation.z() 
		) 
	};  
}

bool check_vec3_OK( Eigen::Vector3d vec ){
	return !( isnan( vec.x() ) || isnan( vec.y() ) || isnan( vec.z() ) ) &&
		   !( isinf( vec.x() ) || isinf( vec.y() ) || isinf( vec.z() ) );
}

bool check_quat_OK( Eigen::Quaterniond quat ){
	return !( isnan( quat.w() ) || isnan( quat.x() ) || isnan( quat.y() ) || isnan( quat.z() ) ) &&
		   !( isinf( quat.w() ) || isinf( quat.x() ) || isinf( quat.y() ) || isinf( quat.z() ) );
}

bool check_dbbl_OK( double number ){  return !( isnan( number ) || isinf( number ) );  }

bool check_matx_OK( Eigen::MatrixXd matx ){
	size_t numRows = matx.rows() , 
		   numCols = matx.cols() ; 
	bool rtnOK = true;
	for( size_t i = 0 ; i < numRows ; i++ ){
		for( size_t j = 0 ; j < numCols ; j++ ){
			rtnOK = check_dbbl_OK( matx( i , j ) );
			if( !rtnOK ){  break;  };
		}
		if( !rtnOK ){  break;  };
	}
	return rtnOK;
}

bool check_pose_OK( Pose_ASP pose ){  
	return check_vec3_OK( pose.position ) && check_quat_OK( pose.orientation ) && check_matx_OK( pose.homogMatx );
}

Eigen::MatrixXd closest_pair_in_hits_to( const RayHits& graspPairs , const Eigen::Vector3d& point ){
	// Find the closest grasp pair to the given point
	// NOTE: This function expects evenly-matched pairs, as produced by 'perforate_meshes_and_obtain_FILO_pairs'
	Eigen::MatrixXd rtnPair = Eigen::MatrixXd ::Zero( 2 , 3 );
	// 1. Check for evenly-matched pairs
	if( graspPairs.enter.rows() == graspPairs.exit.rows() ){
		size_t len = graspPairs.enter.rows();
		Eigen::MatrixXd centers = Eigen::MatrixXd ::Zero( len , 3 );
		Eigen::MatrixXd A , B , cen;
		// 2. Get all of the grasp centers
		for( size_t i = 0 ; i < len ; i++ ){
			A   = graspPairs.enter.row(i);
			B   = graspPairs.exit.row(i);
			cen = ( A + B ) / 2.0;
			centers.row(i) = cen;
		}
		// 3. Get the index of the closest center
		IndexDbblResult result = closest_point_to_sq( centers , point );
		rtnPair.row(0) = graspPairs.enter.row( result.index );
		rtnPair.row(1) = graspPairs.exit.row(  result.index );
	}else{
		// N. else pairs are not evenly matched, return error points
		rtnPair.row(0) = err_vec3();
		rtnPair.row(1) = err_vec3();
	}
	return rtnPair;
}

double least_dist_from_pnt_to_row( const Eigen::Vector3d& query , const Eigen::MatrixXd& pnts ){
	
	bool SHOWDEBUG = false; // if( SHOWDEBUG ){  cout << "" << endl;  }
	
	size_t len = pnts.rows();
	if( SHOWDEBUG ){  cout << "\t\t\tThere are " << len << " points to investigate" << endl;  }
	double leastDist = nan("") ,
		   currDist  = 0.0     ;
	Eigen::Vector3d currPnt;
	if( len > 0 ){  leastDist = BILLION_D;  }
	for( size_t i = 0 ; i < len ; i++ ){
		currPnt   = pnts.row(i);
		currDist  = ( currPnt - query ).norm();
		leastDist = min( leastDist , currDist );
	}
	if( SHOWDEBUG ){  cout << "\t\t\tLeast distance: "  << leastDist << endl;  }
	return leastDist;
}

double greatest_dist_from_pnt_to_row( const Eigen::Vector3d& query , const Eigen::MatrixXd& pnts ){	
	size_t len = pnts.rows();
	double grtstDist = nan("") ,
		   currDist  = 0.0     ;
	Eigen::Vector3d currPnt;
	if( len > 0 ){  grtstDist = -BILLION_D;  }
	for( size_t i = 0 ; i < len ; i++ ){
		currPnt   = pnts.row(i);
		currDist  = ( currPnt - query ).norm();
		grtstDist = max( grtstDist , currDist );
	}
	return grtstDist;
}

double least_dist_from_pnt_to_hit( const Eigen::Vector3d& query , const RayHits& hits ){
	return min(  
		least_dist_from_pnt_to_row( query , hits.enter ) ,
		least_dist_from_pnt_to_row( query , hits.exit  )
	);
}

double greatest_dist_from_pnt_to_hit( const Eigen::Vector3d& query , const RayHits& hits ){
	return max(  
		greatest_dist_from_pnt_to_row( query , hits.enter ) ,
		greatest_dist_from_pnt_to_row( query , hits.exit  )
	);
}

// __ End Helpers __


// == Printing Helpers == 

void print_vec3d_inline( Eigen::Vector3d vec ){ cout << "( " << vec(0) << " , " << vec(1) << " , " << vec(2) << " )"; }
void print_vec2d_inline( Eigen::Vector2d vec ){ cout << "( " << vec(0) << " , " << vec(1) << " )"; }

string Eig_vec3d_to_str( Eigen::Vector3d& vec ){
	return "[ " + to_string( vec(0) ) + " , " + to_string( vec(1) ) + " , " + to_string( vec(2) ) + " ]";
}

string Eig_Quatd_to_str( Eigen::Quaterniond& quat ){
	return "[ w: " + to_string( quat.w() ) + " , x: " + to_string( quat.x() ) + " , y: " + to_string( quat.y() ) + 
	      " , z: " + to_string( quat.z() ) + " ]";
}

std::ostream& operator<<( std::ostream& os , const Eigen::Vector3d& vec ){ 
	os << "[ " << vec(0) <<  " , " << vec(1) <<  " , " << vec(2) << " ]";
	return os;
}

std::ostream& operator<<( std::ostream& os , const Eigen::Vector2d& vec ){ 
	os << "[ " << vec(0) <<  " , " << vec(1) << " ]";
	return os;
}

std::ostream& operator<<( std::ostream& os , const Eigen::Quaterniond& quat ){
	os << "[ w: " << quat.w() << " , x: " << quat.x() << " , y: " << quat.y() << " , z: " << quat.z() << " ]";
	return os;
}

std::ostream& operator<<(std::ostream& os, const Segment2D_ASP& segment){
	os << "[ ";
		os << "( " << segment.pnt1(0) << " , " << segment.pnt1(1) << " ) , ( " << segment.pnt2(0) << " , " << segment.pnt2(1) << " )";
	os << " ]";
	return os; // You must return a reference to the stream!
}

std::ostream& operator<<( std::ostream& os , const Pose_ASP& pose ){
	os << "(Pose_ASP){ " << pose.position << " , " << pose.orientation << " }";
	return os; // You must return a reference to the stream!
}

string matx_1x3_to_string( double rowVec[3] ){
	string rtnStr = "";
	rtnStr += "[ " + to_string( rowVec[0] ) + " , " + to_string( rowVec[1] ) + " , " + to_string( rowVec[2] ) + " ]";
	return rtnStr;
}

string matx_3x3_to_string( double matx[3][3] ){
	string rtnStr = "";
	rtnStr += "[ " + matx_1x3_to_string( matx[0] ) + " , \n"
		   +  "  " + matx_1x3_to_string( matx[1] ) + " , \n"
		   +  "  " + matx_1x3_to_string( matx[2] ) + " ]";
	return rtnStr;
}

// __ End Printing __


// == Memory Helpers ==

Eigen::Vector3d Vec3D_from_aligned_stdvec( std::vector< Eigen::Vector3d , Eigen::aligned_allocator< Eigen::Vector3d > > vectorVector , 
										   size_t index ){
	// This is a utility function to extract an aligned 'Vector3d' from a std::vector and store it in a regular 'Vector3d'
	Eigen::Vector3d rtnVec;
	size_t i = 0;
	for( i = 0 ; i < 3 ; i++ ){ // Extract the standard Eigen struct from the goofy aligned one
		rtnVec( i ) = vectorVector[ index ]( i );
	}
	return rtnVec;
}

Eigen::MatrixXd vstack( const Eigen::MatrixXd& A , const Eigen::MatrixXd& B ){
	// URL , Stack two matrices vertically: https://stackoverflow.com/a/21496281
	// NOTE: This function assumes that 'A' and 'B' have the same number of columns
	size_t Alen = A.rows() ,
		   Blen = B.rows() ;
	if( Alen < 1 ){  return B;  } // If either matrix is empty , return the other
	if( Blen < 1 ){  return A;  }
	Eigen::MatrixXd rtnMatx( Alen + Blen , A.cols() );
	rtnMatx << A , 
			   B ;
	return rtnMatx;
}

Eigen::MatrixXi vstack( const Eigen::MatrixXi& A , const Eigen::MatrixXi& B ){
	// URL , Stack two matrices vertically: https://stackoverflow.com/a/21496281
	// NOTE: This function assumes that 'A' and 'B' have the same number of columns
	size_t Alen = A.rows() ,
		   Blen = B.rows() ;
	if( Alen < 1 ){  return B;  } // If either matrix is empty , return the other
	if( Blen < 1 ){  return A;  }
	Eigen::MatrixXi rtnMatx( Alen + Blen , A.cols() );
	rtnMatx << A , 
			   B ;
	return rtnMatx;
}

Eigen::MatrixXd copy_V_plus_row( const Eigen::MatrixXd& pMatx , const Eigen::Vector3d& nuVec ){ 
	// Extend vertices list by 1 R3 vector
	// NOTE: This function assumes that 'pMatx' is either empty or has 3 columns
	// NOTE: This function is not efficient
	
	bool SHOWDEBUG = false;
	
	Eigen::MatrixXd rtnMatx;
	size_t pRows = pMatx.rows();
	if( SHOWDEBUG ){  cout << "Found a matrix with " << pRows << " rows" << endl;  }
	if( pRows < 1 ){
		rtnMatx = Eigen::MatrixXd::Zero(1,3);
		rtnMatx.row(0) = nuVec;
	}else{
		rtnMatx = Eigen::MatrixXd::Zero( pRows+1 , 3 );
		rtnMatx.block( 0 , 0 , pRows , 3 ) = pMatx;
		rtnMatx.row( pRows ) = nuVec;
	}
	
	if( SHOWDEBUG ){
		cout << "About to return expanded matrix ..." << endl;
		cout << rtnMatx << endl;
	}
	
	return rtnMatx;
}

Eigen::MatrixXd copy_V_plus_row( const Eigen::MatrixXd& pMatx , const Eigen::Vector2d& nuVec ){ 
	// Extend vertices list by 1 R2 vector , return copy
	Eigen::MatrixXd rtnMatx;
	size_t pRows = pMatx.rows();
	if( pRows < 1 ){
		rtnMatx = Eigen::MatrixXd::Zero(1,2);
		rtnMatx.row(0) = nuVec;
	}else{
		rtnMatx = Eigen::MatrixXd::Zero( pRows+1 , 2 );
		rtnMatx.block( 0 , 0 , pRows , 3 ) = pMatx;
		rtnMatx.row( pRows ) = nuVec;
	}
	
	return rtnMatx;
}

Eigen::MatrixXi copy_F_plus_row( const Eigen::MatrixXi& pMatx , const Eigen::Vector3i& nuVec ){
	// Extend vertices list by 1 I3 vector , return copy
	Eigen::MatrixXi rtnMatx;
	size_t pRows = pMatx.rows();
	if( pRows < 1 ){
		rtnMatx = Eigen::MatrixXi::Zero(1,3);
		rtnMatx.row(0) = nuVec;
	}else{
		rtnMatx = Eigen::MatrixXi::Zero( pRows+1 , 3 );
		rtnMatx.block( 0 , 0 , pRows , 3 ) = pMatx;
		rtnMatx.row( pRows ) = nuVec;
	}
	return rtnMatx;
}

Eigen::MatrixXd copy_column_plus_dbbl( const Eigen::MatrixXd& columnMatx , double nuNum ){ 
	// Extend column by 1 number , return copy
	Eigen::MatrixXd rtnMatx;
	size_t pRows = columnMatx.rows();
	if( pRows < 1 ){
		rtnMatx = Eigen::MatrixXd::Zero(1,1);
		rtnMatx.row(0) << nuNum;
	}else{
		rtnMatx = Eigen::MatrixXd::Zero( pRows+1 , 1 );
		rtnMatx.block( 0 , 0 , pRows , 1 ) = columnMatx;
		rtnMatx.row( pRows ) << nuNum;
	}
	return rtnMatx;
}

Eigen::MatrixXd copy_V_minus_row( const Eigen::MatrixXd& pMatx , size_t rowNum ){
    
    bool SHOWDEBUG = false; // if( SHOWDEBUG ){  cout << "" << endl;  }
    
    size_t nRows = pMatx.rows() , 
           count = 0 ;
    if( SHOWDEBUG ){  cout << "There are " << nRows << " rows. Copying with excluded row " << rowNum << " ..." << endl;  }
    if( rowNum < nRows ){
        Eigen::MatrixXd rtnMatx;
        if( nRows > 1 ){
            rtnMatx = Eigen::MatrixXd::Zero( pMatx.rows() - 1 , pMatx.cols() );
            for( size_t i = 0 ; i < nRows ; i++ ){
                if( i != rowNum ){
                    rtnMatx.row( count ) = pMatx.row(i);
                    count++;
                }
            }
        }
        if( SHOWDEBUG ){  cout << "Return " << endl << rtnMatx << "..." << endl;  }
        return rtnMatx;
    }else{  throw std::out_of_range ( "copy_V_minus_row , Index " + to_string( rowNum )
									+ " out of range [ 0 , " + to_string( nRows-1 ) + " ] !" );   }
}

Segment2D_ASP copy_Segment2D_ASP( Segment2D_ASP original ){ return Segment2D_ASP{ original.pnt1 , original.pnt2 }; }

void copy_Segment2D_vec_from_A_to_B( std::vector<Segment2D_ASP>& A , std::vector<Segment2D_ASP>& B ){
	// Copy all of the elements from A to B // NOTE: This function DESTROYS all elements of B before copy!
	B.clear(); // Destroy elements of B
	size_t lenA = A.size();
	for( size_t i = 0 ; i < lenA ; i++ ){  B.push_back( copy_Segment2D_ASP( A[i] ) );  }
}

void copy_Segment2D_vec_from_A_to_B( const std::vector<Segment2D_ASP>& A , std::vector<Segment2D_ASP>& B ){
	// Copy all of the elements from A to B // NOTE: This function DESTROYS all elements of B before copy!
	B.clear(); // Destroy elements of B
	size_t lenA = A.size();
	for( size_t i = 0 ; i < lenA ; i++ ){  B.push_back( copy_Segment2D_ASP( A[i] ) );  }
}

TriMeshVFN_ASP copy_TriMeshVFN_ASP( TriMeshVFN_ASP& original ){
	TriMeshVFN_ASP rtnStruct;
	rtnStruct.V      = original.V;
	rtnStruct.F      = original.F;
	rtnStruct.N      = original.N;
	rtnStruct.center = original.center; 
	rtnStruct.axis   = original.axis; 
	rtnStruct.type   = original.type;
	return rtnStruct;
}

TriMeshVFN_ASP copy_TriMeshVFN_ASP( const TriMeshVFN_ASP& original ){
	TriMeshVFN_ASP rtnStruct;
	rtnStruct.V      = original.V;
	rtnStruct.F      = original.F;
	rtnStruct.N      = original.N;
	rtnStruct.center = original.center; 
	rtnStruct.axis   = original.axis; 
	rtnStruct.type   = original.type;
	return rtnStruct;
}

Eigen::MatrixXd load_vec_vec_into_matx( std::vector<std::vector<double>>& vecVec ){
	// Transform a vector of vectors into a matrix
	// NOTE: This function assumes that 'vecVec' has at least one vector with at least one element
	// NOTE: This function assumes that 'vecVec' is rectangular
	size_t numRows = vecVec.size()    ,
		   numCols = vecVec[0].size() ;
	Eigen::MatrixXd rtnMatx = Eigen::MatrixXd::Zero( numRows , numCols );
	for( size_t i = 0 ; i < numRows ; i++ ){
		for( size_t j = 0 ; j < numCols ; j++ ){
			rtnMatx( i , j ) = vecVec[i][j];
		}
	}
	return rtnMatx;
}

Eigen::MatrixXd repeat_vector3d( Eigen::Vector3d origVec , size_t N ){
	Eigen::MatrixXd rtnMatx = Eigen::MatrixXd::Zero( N , 3 );
	for( size_t i = 0 ; i < N ; i++ ){  rtnMatx.row(i) = origVec;  }
	return rtnMatx;
}

std::vector<double> vec_nan( size_t N ){
	// Return a double vector of NaN with specified length
	std::vector<double> rtnVec;
	for( size_t i = 0 ; i < N ; i++ ){  rtnVec.push_back( nan("") );  }
	return rtnVec;
}
	 

// __ End Memory __


// == Geo 2D ==

double d_point_to_segment_2D( Eigen::Vector2d point , Segment2D_ASP segment ){
    // Return the shortest (perpendicular) distance between 'point' and a line 'segment' 
    // URL: http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
    Eigen::Vector2d segPt1 = segment.pnt1; 
    Eigen::Vector2d segPt2 = segment.pnt2;
    return abs( ( segPt2(0) - segPt1(0) ) * ( segPt1(1) - point(1)  ) - 
                ( segPt1(0) - point(0)  ) * ( segPt2(1) - segPt1(1) ) ) / 
                  sqrt( pow( segPt2(0) - segPt1(0) , 2 ) + pow( segPt2(1) - segPt1(1) , 2 ) );
}

// Test if Eigen 2D vectors are "about equal"
bool vec2d_eq( Eigen::Vector2d& v1 , Eigen::Vector2d& v2 ){ return ( v1 - v2 ).norm() <= EPSILON; }


double winding_num( Eigen::Vector2d& point , Eigen::MatrixXd& polygon ){
    // Find the winding number of a point with respect to a polygon , works for both CW and CCWpoints
    //  This algorithm is translation invariant, and can handle convex, nonconvex, and polygons with crossing sides.
	//  This works by shifting the point to the origin (preserving the relative position of the polygon points), and 
    // tracking how many times the positive x-axis is crossed
    
    /* NOTE: Function assumes that the points of 'polygon' are ordered. Algorithm does not work if they are not 
	   NOTE: This algorithm does NOT handle the case when the point lies ON a polygon side. For this problem it is assumed that
		     there are enough trial points to ignore this case */
    
    double w   = 0; 
	int    len = polygon.rows();
	Eigen::MatrixXd transformedPoly = Eigen::MatrixXd::Zero( len , 2 );
	Eigen::Vector2d polyPnt;
	Eigen::Vector2d ofstPnt;
    
    //~ for vertex in polygon: # Shift the point to the origin, preserving its relative position to polygon points
    for( int i = 0 ; i < len ; i++ ){ 
        //~ v_i.append( np.subtract( vertex , point ) )
        polyPnt = polygon.row( i );
        ofstPnt = polyPnt - point;
        transformedPoly.row( i ) = ofstPnt;
	}
	
	double x_i = 0.0;  double x_i1 = 0.0;
	double y_i = 0.0;  double y_i1 = 0.0;
	
    //~ for i in range(len(v_i)): # for each of the transformed polygon points, consider segment v_i[i]-->v_i[i+1]
    for( int i = 0 ; i < len ; i++ ){ 
    
		x_i  = transformedPoly( indexw( len , i   ) , 0 );
		x_i1 = transformedPoly( indexw( len , i+1 ) , 0 );
		y_i  = transformedPoly( indexw( len , i   ) , 1 );  
		y_i1 = transformedPoly( indexw( len , i+1 ) , 1 );  
        
        // NOTE: This function ALWAYS excludes colinear points, whether this bit is set or not
        if( eq( 0.0 , 
				d_point_to_segment_2D( Eigen::Vector2d{ 0 , 0 } , 
									   Segment2D_ASP{ Eigen::Vector2d{ x_i  , y_i  } , 
													  Eigen::Vector2d{ x_i1 , y_i1 } } ) ) ){
            return 0.0;
		}
		
		double r = 0.0;
		
        if( y_i * y_i1 < 0 ){ // if the segment crosses the x-axis
            r = x_i + ( y_i * ( x_i1 - x_i ) ) / ( y_i - y_i1 ); // location of x-axis crossing
            if( r > 0 ){ // positive x-crossing
                if( y_i < 0 ){
                    w += 1; // CCW encirclement
                } else {
                    w -= 1; //  CW encirclement
				}
			}
        // If one of the polygon points lies on the x-axis, we must look at the segments before and after to determine encirclement
        } else if( eq( y_i , 0 ) && ( x_i > 0 ) ){ // on the positive x-axis, leaving possible crossing
            if( y_i1 > 0 ){
                w += 0.5; // possible CCW encirclement or switchback from a failed CW crossing
            } else {
                w -= 0.5; // possible CW encirclement or switchback from a failed CCW crossing
			}
        } else if( ( eq( y_i1 , 0.0 ) ) && ( x_i1 > 0.0 ) ){ // on the positive x-axis, approaching possible crossing
            if( y_i < 0 ){
                w += 0.5; // possible CCW encirclement pending
            } else {
                w -= 0.5; // possible  CW encirclement pending
			}
		}
	}
    return w;
}

//~ def point_in_poly_w( point , polygon ): 
bool point_in_poly_w( Eigen::Vector2d& point , Eigen::MatrixXd& polygon , bool makeCycle ){
    // Return True if the 'polygon' contains the 'point', otherwise return False, based on the winding number
    if( makeCycle ){
		Eigen::Vector2d firstPoint = polygon.row(0);
		Eigen::MatrixXd comparePoly = copy_V_plus_row( polygon , firstPoint );
		return !eq( winding_num( point , comparePoly ) , 0.0 ); // The winding number gives the number of times a polygon encircles a point 
	}else{
		return !eq( winding_num( point , polygon ) , 0.0 ); // The winding number gives the number of times a polygon encircles a point 
	}
}

bool point_in_triangle_2D( Eigen::Vector2d pt , Eigen::Vector2d v0 , Eigen::Vector2d v1 , Eigen::Vector2d v2 ){
	// Return True if 'p' lies within a triangle defined by 'v0' , 'v1' , 'v2'
	// URL , Point in triangle: https://stackoverflow.com/a/34093754/893511
	double dX   = pt(0) - v2(0);
    double dY   = pt(1) - v2(1);
    double dX21 = v2(0) - v1(0);
    double dY12 = v1(1) - v2(1);
    double D    = dY12 * ( v0(0) - v2(0) ) + dX21 * ( v0(1) - v2(1) );
    double s    = dY12 * dX + dX21 * dY;
    double t    = ( v2(1) - v0(1) ) * dX + ( v0(0) - v2(0) ) * dY;
    if( D < 0 ){ return ( s <= 0 ) && ( t <= 0 ) && ( s + t >= D ); }
    else{ return ( s >= 0 ) && ( t >= 0 ) && ( s + t <= D ); }
}

bool point_in_triangle_2D( Eigen::Vector2d pt , Eigen::MatrixXd triPtsCCW ){
	Eigen::Vector2d p0 = triPtsCCW.row(0);
	Eigen::Vector2d p1 = triPtsCCW.row(1);
	Eigen::Vector2d p2 = triPtsCCW.row(2);
	return point_in_triangle_2D( pt , p0 , p1 , p2 );
}

size_t index_of_segment_with_pnt1( std::vector<Segment2D_ASP> segmentList , Eigen::Vector2d& queryPnt1 ){
	// Return the index of the segment that begins with 'queryPnt1'
	size_t vecLen = segmentList.size();
	for( size_t i = 0 ; i < vecLen ; i++ ){
		if( vec2d_eq( queryPnt1 , segmentList[i].pnt1 ) ){ return i; } // If there is a match , return the index
	}
	return vecLen; // Otherwise return an invalid index
}

std::vector<Segment2D_ASP> segment_list_to_segment_vector( std::list<Segment2D_ASP> segLst ){
	std::vector<Segment2D_ASP> rtnLst;
	std::list<Segment2D_ASP>::const_iterator iterator;
	for( iterator = segLst.begin() ; iterator != segLst.end() ; ++iterator) {
		rtnLst.push_back( copy_Segment2D_ASP( *iterator ) );
	}
	return rtnLst;
}

std::list<Segment2D_ASP> order_RH_border_segments( std::vector<Segment2D_ASP> segmentList ){
	// Return a copy of 'segmentList' that is reordered into the CCW border of a polygon

	size_t i         = 0                  ,
	       numSegs   = segmentList.size() ,
	       result    = 0                  , 
	       usedCount = 0                  ; 

	//~ segCpy = segmentList[:] # Copy the segment list to safely modify
	std::vector<bool> usedElems = bool_false_vector( segmentList.size() ); // Use this to signify which elems have been used
    
    std::list< Segment2D_ASP > rtnSeq;
    //~ rtnSeq = [ segCpy.pop( 0 ) ] # pop the first segment for the polygon , Always pop this one for consistency
    rtnSeq.push_back( copy_Segment2D_ASP( segmentList[0] ) );
    usedElems[0] = true;
    usedCount++;
    
    Eigen::Vector2d bgnPnt;
    
    //~ while( len( segCpy ) ):
    while( usedCount < numSegs ){
        //~ bgnPnt = rtnSeq[-1][1]
        bgnPnt = rtnSeq.back().pnt2;
        //~ result = pop_segment_with_bgn( segCpy , bgnPnt )
        result = index_of_segment_with_pnt1( segmentList , bgnPnt );
        //~ if result == None:
        if( result >= numSegs )
            break;
        //~ rtnSeq.append( result )
        rtnSeq.push_back( copy_Segment2D_ASP( segmentList[result] ) );
		usedElems[result] = true;
		usedCount++;
	}
    
    //~ if len( segCpy ) > 0:
    if( usedCount < numSegs )
        //~ warn( "polygon_from_RH_border_segments: Could not construct a simple polygon using all segments" )
        cout << "WARN! , order_RH_border_segments: Could not construct a simple polygon using all segments" << endl;
        
    //~ if rtnSeq[0][0] != rtnSeq[-1][1]:
    if( !vec2d_eq( rtnSeq.front().pnt1 , rtnSeq.back().pnt2 ) )
        //~ warn( "polygon_from_RH_border_segments: Could not construct a closed polygon" )
        cout << "WARN! , order_RH_border_segments: Could not construct a closed polygon" << endl;
        
    return rtnSeq;
}

Eigen::MatrixXd Matx2D_points_to_Matx3D_flat( Eigen::MatrixXd& pnts2D ){
	// Given a Nx2 matrix of 2D points 'pnts2D' in which each row is a point , Return a Nx3 with all z_i = 0
	size_t len = pnts2D.rows();
	Eigen::MatrixXd rtnMatx3D = Eigen::MatrixXd::Zero( len , 3 );
	for( size_t i = 0 ; i < len ; i++ ){
		// X                                 ||   Y                                 ||   Z
		rtnMatx3D( i , 0 ) = pnts2D( i , 0 );  rtnMatx3D( i , 1 ) = pnts2D( i , 1 );  rtnMatx3D( i , 2 ) = 0;  
	}
	return rtnMatx3D;
}



bool intersect_seg_2D( Segment2D_ASP seg1 , Segment2D_ASP seg2 , bool includeEndpoints ){
    // Return true if line segments 'seg1' and 'seg2' intersect, otherwise false 
    // URL: http://www-cs.ccny.cuny.edu/~wolberg/capstone/intersection/Intersection%20point%20of%20two%20lines.html
    // NOTE: 'uA' and 'uB' could be used to calc intersection point if desired, see above URL
    // NOTE: This function will return true if either of the segments are of zero-length!
    double den   = (seg2.pnt2(1)-seg2.pnt1(1)) * (seg1.pnt2(0)-seg1.pnt1(0)) - (seg2.pnt2(0)-seg2.pnt1(0)) * (seg1.pnt2(1)-seg1.pnt1(1));
    double uAnum = (seg2.pnt2(0)-seg2.pnt1(0)) * (seg1.pnt1(1)-seg2.pnt1(1)) - (seg2.pnt2(1)-seg2.pnt1(1)) * (seg1.pnt1(0)-seg2.pnt1(0));
    double uBnum = (seg1.pnt2(0)-seg1.pnt1(0)) * (seg1.pnt1(1)-seg2.pnt1(1)) - (seg1.pnt2(1)-seg1.pnt1(1)) * (seg1.pnt1(0)-seg2.pnt1(0));
    if( eq( den , 0.0 ) ){ // If the denominator for the equations for u_a and u_b is 0 then the two lines are parallel. 
		// If the denominator and numerator for the equations for u_a and u_b are 0 then the two lines are coincident
        if ( eq( uAnum , 0.0 ) && eq( uBnum , 0.0 ) ){  
			double seg1len = ( seg1.pnt2 - seg1.pnt1 ).norm();
			double projLenBgn = 0 , 
				   projLenEnd = 0 ;
			Eigen::Vector2d seg1dir = ( seg1.pnt2 - seg1.pnt1 ).normalized();
			
			projLenBgn = seg1dir.dot( seg2.pnt1 - seg1.pnt1 );
			projLenEnd = seg1dir.dot( seg2.pnt2 - seg1.pnt1 );
			
			if( ( !includeEndpoints && ( projLenBgn > 0 && projLenBgn < seg1len ) || ( projLenEnd > 0 && projLenEnd < seg1len ) ) ||
			    ( includeEndpoints && ( projLenBgn >= 0 && projLenBgn <= seg1len ) || ( projLenEnd >= 0 && projLenEnd <= seg1len ) ) )
			    return true;  
			else
				return false;
		}else{  return false;  }
	}else{
        double uA = uAnum * 1.0 / den;
        double uB = uBnum * 1.0 / den;
        if( ( includeEndpoints && ( uA >= 0.0 && uA <= 1.0 ) && ( uB >= 0 && uB <= 1 ) ) 
            || ( ( !includeEndpoints ) && ( uA > 0.0 && uA < 1.0 ) && ( uB > 0.0 && uB < 1.0 ) ) ){  return true;  }
        else{  return false;  }
	}
}

std::vector<Segment2D_ASP> segments_from_poly2D_matx( Eigen::MatrixXd& polyPts2D , bool closed ){
	// Convert a polygon specified as row list of points into a vector of segments
	// NOTE: This function assumes that the vertices are listed in order (CCW or CW) such that they form a simple polygon , not enforced
	size_t len = polyPts2D.rows(); // Number of points stored in the matrix
	std::vector<Segment2D_ASP> rtnVec;
	for( size_t i = 0 ; i < len-1 ; i++ ){
		//~ Eigen::Vector2d one = polyPts2D.row( i   );
		//~ Eigen::Vector2d two = polyPts2D.row( i+1 );
		//~ rtnVec.push_back( Segment2D_ASP{ one , two } )
		rtnVec.push_back( Segment2D_ASP{ Eigen::Vector2d( polyPts2D.row( i ) ) , Eigen::Vector2d( polyPts2D.row( i+1 ) ) } );
	}
	if( closed ){ rtnVec.push_back(  Segment2D_ASP{ Eigen::Vector2d( polyPts2D.row( len-1 ) ) , 
												    Eigen::Vector2d( polyPts2D.row( 0     ) ) }  ); }
	return rtnVec;
}

bool polygon_collide_2D( Eigen::MatrixXd& polyPtsA , Eigen::MatrixXd& polyPtsB , bool endpoints , bool makeCycle ){
	// NOTE: This function assumes that polygons are closed
	std::vector<Segment2D_ASP> polySegsA = segments_from_poly2D_matx( polyPtsA , makeCycle ); 
	std::vector<Segment2D_ASP> polySegsB = segments_from_poly2D_matx( polyPtsB , makeCycle ); 
	bool DEBUG = false;
	size_t lenA = polySegsA.size() , 
	       lenB = polySegsB.size() ;
	// 1. Return true if any of the segments intersect
	for( size_t i = 0 ; i < lenA ; i++ ){
		for( size_t j = 0 ; j < lenB ; j++ ){
			if( DEBUG ){  cout << "Check intersection: " << polySegsA[i] << " and " << polySegsB[j] << endl;  }
			if( intersect_seg_2D( polySegsA[i] , polySegsB[j] , endpoints ) ){  
				if( DEBUG ){  cout << "polygon_collide_2D: Segment Collision!" << endl;  }
				return true;  
			}
		}
	}
	// 2. Return true if any of the points of one are fully contained by the other ( only need to check one )
	Eigen::Vector2d firstA = polyPtsA.row(0);  
	Eigen::Vector2d firstB = polyPtsB.row(0);
	if( point_in_poly_w( firstA , polyPtsB ) ){  
		if( DEBUG ){  cout << "polygon_collide_2D: First point of A insided B!" << endl;  }
		return true;  
	}
	if( point_in_poly_w( firstB , polyPtsA ) ){  
		if( DEBUG ){  cout << "polygon_collide_2D: First point of B insided A!" << endl;  }
		return true;  
	}
	// 3. Return false if all checks negative
	return false;
}

string verts2D_to_string( Eigen::MatrixXd& V ){
	// Return a string representing a list of vertices that is 
	stringstream rtnStr;
	size_t len = V.rows();
	for( size_t i = 0 ; i < len ; i++ ){  rtnStr << V( i , 0 ) << " , " << V( i , 1 ) << endl;  }
	return rtnStr.str();
}

Eigen::MatrixXd tile_triangle_2D( Eigen::Vector2d v0 , Eigen::Vector2d v1 , Eigen::Vector2d v2 , size_t N ){
	//~ """ Tile a traingle with 'N' rows of evenly-spaced points , each row i containing i points , total = Nth triangular number """
    //~ # NOTE: The points returned by this function do not occupy Voronoi cells of equal area, but this is deemed an appropriately-close hack
    
    bool SHOWDEBUG = false;
    
    size_t totalPts = triangular_num( N ) , 
		   pntCount = 0                   ; 
		   
	if( SHOWDEBUG ){  cout << "There will be " << totalPts << " total points." << endl;  }
		   
	Eigen::Vector2d backPos;  backPos << 0 , 0 ;
    //~ rtnPts = []
    Eigen::MatrixXd rtnPts = Eigen::MatrixXd::Zero( totalPts , 2 );
    //~ base = np.subtract( v1 , v0 )
    Eigen::Vector2d base = v1 - v0;
    //~ back = np.subtract( v2 , v0 )
    Eigen::Vector2d back = v2 - v0;
    //~ # 1. Get base direction
    //~ baseDir = vec_unit( base )
    Eigen::Vector2d baseDir = base.normalized();
    //~ # 2. Get base length
    //~ baseLen = vec_mag( base )
    double baseLen = base.norm();
    //~ baseSeg = baseLen / ( N + 1 )
    double baseSeg = baseLen / ( (double)N + 1 );
    //~ # 3. Get backbone direction
    //~ backDir = vec_unit( back )
    Eigen::Vector2d backDir = back.normalized();
    //~ # 4. Get backbone length
    //~ backLen = vec_mag( back )
    double backLen = back.norm();
    //~ backSeg = backLen / ( N + 1 )
    double backSeg = backLen / ( (double)N + 1 );
    //~ # 5. for i = N to 1
    //~ for i in xrange( 1 , N+3 ):
    for( size_t i = 1 ; i < N+3 ; i++ ){
        //~ # 6. Get the backbone position
        //~ backPos = np.add( v0 , np.multiply( backDir , (i-1) * backSeg + backSeg * 0.75 ) )
        backPos = v0 + backDir * ( ((double)i-1) * backSeg + backSeg * 0.50 );
        //~ # 7. Get the row length
        //~ # 8. Get i evenly-spaced points on the row and append to the total list
        //~ for j in xrange( 1 , N-i+2 ):
        for( size_t j = 1 ; j < N-i+2 ; j++ ){
			
			if( SHOWDEBUG ){  cout << "( i , j ): ( " << i << " , " << j << " ): " << pntCount << endl;  }
			
            //~ rtnPts.append(  np.add( backPos , np.multiply( baseDir , (j-1) * baseSeg + baseSeg * 0.75  ) )  )
            rtnPts.row( pntCount ) = backPos + baseDir * ( ((double)j-1) * baseSeg + baseSeg * 0.75 );
            pntCount++;
		}
	}
    //~ # 9. Return the total list of points
    return rtnPts;
}

Eigen::MatrixXd tile_triangle_2D( Eigen::MatrixXd CCWverts , size_t N ){
	// Tile a traingle with 'N' rows of evenly-spaced points , each row i containing i points , total = Nth triangular number """
	Eigen::Vector2d p0 = CCWverts.row(0);
	Eigen::Vector2d p1 = CCWverts.row(1);
	Eigen::Vector2d p2 = CCWverts.row(2);
	return tile_triangle_2D( p0 , p1 , p2 , N );
}

double tri_area_2D( Eigen::Vector2d p0 , Eigen::Vector2d p1 , Eigen::Vector2d p2 ){ 
	// Return area of CCW triangle in R2
	double Ax = p0(0);  double Ay = p0(1);
	double Bx = p1(0);  double By = p1(1);
	double Cx = p2(0);  double Cy = p2(1);
	return abs( ( Ax * ( By - Cy ) + Bx * ( Cy - Ay ) + Cy * ( Ay - By ) ) 
				/ 2.0 );
}

double tri_area_2D( Eigen::MatrixXd CCWverts ){ 
	// Return area of CCW triangle in R2
	Eigen::Vector2d p0 = CCWverts.row(0);
	Eigen::Vector2d p1 = CCWverts.row(1);
	Eigen::Vector2d p2 = CCWverts.row(2);
	return tri_area_2D( p0 , p1 , p2 );
}

double overlapping_tri_area( Eigen::MatrixXd tri1 , Eigen::MatrixXd tri2 , size_t discretLvl ){ 
	// Estimate the area shared by the two triangles
	// NOTE: T(14) = 105 is the first triangular number > 100
	bool SHOWDEBUG = false;
	// 1. If the traingles do not collide, Then there is no shared area 
	if( polygon_collide_2D( tri1 , tri2 , false , true ) ){
		if( SHOWDEBUG ){  cout << "Triangles Collide ..." << endl;  }
		Eigen::Vector2d currPnt;
		// 1. Get the area of the first triangle ( Shared area cannot be greater than either single triangle )
		double tri1area    = tri_area_2D( tri1 );
		if( SHOWDEBUG ){  cout << "Triangle 1 area: " << tri1area << endl;  }
		size_t insideCount = 0;
		// 2. Tile the first triangle with points and note the number of points
		Eigen::MatrixXd tilePts = tile_triangle_2D( tri1 , discretLvl );
		size_t          numPts  = tilePts.rows();
		if( SHOWDEBUG ){  cout << "Tiled Triangle 1 with " << numPts << " points." << endl;  }
		// 3. For each of the tile points
		for( size_t i = 0 ; i < numPts ; i++ ){
			if( SHOWDEBUG ){  cout << "Point " << i + 1 << " ... ";  }
			currPnt = tilePts.row(i);
			// 4. Test if the tile point lies within the second trianle , If so, then add to count
			if( point_in_triangle_2D( currPnt , tri2 ) ){  
				insideCount++;  
				if( SHOWDEBUG ){  cout << "inside!" << endl;  }
			}else{  if( SHOWDEBUG ){  cout << "outside!" << endl;  }  }
		}
		if( SHOWDEBUG ){  cout << "About to calc overlap ..." << endl;  }
		// 5. Return the fraction of the points times the total area of the first triangle
		return (double) insideCount / numPts * tri1area;
	}else{  return 0.0;  }
}

// __ End 2D __


// == Geo 3D ==

Eigen::Vector3d vec3d_random(){  return Eigen::Vector3d( rand_dbbl() , rand_dbbl() , rand_dbbl() );  }

Eigen::Vector3d vec3d_rand_corners( const Eigen::Vector3d& corner1 , const Eigen::Vector3d& corner2 ){
	Eigen::Vector3d span = corner2 - corner1;
	Eigen::Vector3d sample = vec3d_random();
	return Eigen::Vector3d( corner1(0)+span(0)*sample(0) , corner1(1)+span(1)*sample(1) , corner1(2)+span(2)*sample(2) );
}

double angle_between( const Eigen::Vector3d& vec1 , const Eigen::Vector3d& vec2 ){
	// Get the angle between two R3 vectors , radians
	//~ cout << "DEBUG: vec1: " << endl << vec1 << endl;
	//~ cout << "DEBUG: vec2: " << endl << vec2 << endl;
	double angle = acos( vec1.normalized().dot( vec2.normalized() ) ); // for now assume that there are no special cases
	//~ cout << "DEBUG: Dot Product:         " << vec1.normalized().dot( vec2.normalized() ) << endl;
	//~ cout << "DEBUG: acos( Dot Product ): " << acos( vec1.normalized().dot( vec2.normalized() ) ) << endl;
	if( isnan( angle ) ){
		if( vec1.normalized() == vec2.normalized() ){ return 0.0d; }
		else { return M_PI; }
	} else { return angle; }
}

double angle_between( const Eigen::Quaterniond& quat1 , const Eigen::Quaterniond& quat2 ){
	// Get the angle of the shortest turn between two quaternions , radians
	Eigen::Vector3d vec1 = quat1 * Eigen::Vector3d( 0.0 , 0.0 , 1.0 );
	Eigen::Vector3d vec2 = quat2 * Eigen::Vector3d( 0.0 , 0.0 , 1.0 );
	return angle_between( vec1 , vec2 );
}

double angle_between( const Pose_ASP& pose1 , const Pose_ASP& pose2 ){  return angle_between( pose1.orientation , pose2.orientation );  }

// Return the magnitude component of 'vector' that lies along 'basis'
double vec3_project_mag( const Eigen::Vector3d& vector , const Eigen::Vector3d& basis ){ return vector.dot( basis ) / basis.norm(); }

Eigen::Vector3d pnt_proj_onto_ray( const Eigen::Vector3d& point , const Eigen::Vector3d& rayOrg , const Eigen::Vector3d& rayDir ){
	double mag = vec3_project_mag( point - rayOrg , rayDir );
	return rayOrg + rayDir * mag;
}

double dist_to_plane( Eigen::Vector3d planeNorm , Eigen::Vector3d planePoint , Eigen::Vector3d queryPoint ){
	// 1. Calc the difference between the query point and the plan point
	Eigen::Vector3d diff = queryPoint - planePoint;
	// 2. Project the difference onto the plane norm and return this distance
	return vec3_project_mag( diff , planeNorm ); 
}

Eigen::Vector3d vec_proj_to_plane( Eigen::Vector3d vec , Eigen::Vector3d planeNorm ){
	// URL, projection of a vector onto a plane passing through origin: 
	//  http://www.euclideanspace.com/maths/geometry/elements/plane/lineOnPlane/
    // NOTE: This function assumes 'vec' and 'planeNorm' are expressed in the same bases
	Eigen::Vector3d projDir = ( planeNorm.cross( vec ) ).cross( planeNorm ).normalized();
	double          projMag = vec3_project_mag( vec , projDir );
	return projDir * projMag;
}

Eigen::Vector3d pnt_proj_to_plane( Eigen::Vector3d queryPnt , Eigen::Vector3d planePnt , Eigen::Vector3d normal ){
	// Return a point that is 'queryPnt' projected onto a plane with a given 'normal' and passing through 'planePnt'
	// Compute the vector offset from the arbitrary plane point to the point under scrutiny
	Eigen::Vector3d relPnt = queryPnt - planePnt;
	Eigen::Vector3d offset = vec_proj_to_plane( relPnt , normal );
	return planePnt + offset;
}

Eigen::Vector2d pnt_proj_onto_plane_2D( Eigen::Vector3d queryPnt , 
										Eigen::Vector3d planePnt , Eigen::Vector3d normal , Eigen::Vector3d xBasis ){
	Eigen::Vector2d rtnCoords;  rtnCoords << 0 , 0;
	// 0. Obtain mutually orthogonal basis vectors and ensure that they are normalized
	Eigen::Vector3d yBasis = normal.cross( xBasis ).normalized();
	xBasis = yBasis.cross( normal ).normalized();
	// 1. Subtract the origin
	Eigen::Vector3d diff = queryPnt - planePnt;
	// 2. Project the vector onto each of the components
	rtnCoords(0) = xBasis.dot( diff );  rtnCoords(1) = yBasis.dot( diff );  
	return rtnCoords;
}

Eigen::MatrixXd verts3d_proj_to_plane_2D( Eigen::MatrixXd V , 
										  Eigen::Vector3d planePnt , Eigen::Vector3d normal , Eigen::Vector3d xBasis ){
	size_t len = V.rows();
	Eigen::MatrixXd rtnMatx = Eigen::MatrixXd::Zero( len , 2 );
	Eigen::Vector3d queryPnt;
	Eigen::Vector3d diff;
	// 0. Obtain mutually orthogonal basis vectors and ensure that they are normalized
	Eigen::Vector3d yBasis = normal.cross( xBasis ).normalized();
	xBasis = yBasis.cross( normal ).normalized();
	for( size_t i = 0 ; i < len ; i++ ){
		queryPnt = V.row(i);
		// 1. Subtract the origin
		diff = queryPnt - planePnt;
		// 2. Project the vector onto each of the components
		rtnMatx( i , 0 ) = xBasis.dot( diff );  rtnMatx( i , 1 ) = yBasis.dot( diff );  
	}
	return rtnMatx;
}

Eigen::Vector3d get_average_V( const Eigen::MatrixXd& V ){
	// Return the average of every row of V
	size_t i       = 0        ,
		   numRows = V.rows() ; 
	Eigen::Vector3d rtnVec;  rtnVec << 0 , 0 , 0;
	Eigen::Vector3d temp;
	for( i = 0 ; i < numRows ; i++ ){ 
		temp = V.row(i);
		rtnVec += temp; 
	}
	return rtnVec / numRows;
}

Eigen::Vector3d selected_average_V( Eigen::MatrixXd& V , std::vector<size_t>& selection ){
	// Return the average of the selected points
	size_t len = selection.size();
	Eigen::MatrixXd selectRows = Eigen::MatrixXd::Zero( len , 3 );
	for( size_t i = 0 ; i < len ; i++ ){  selectRows.row(i) = V.row( selection[i] );  }
	return get_average_V( selectRows );
}

void shift_V_inplace( Eigen::MatrixXd& V , const Eigen::Vector3d& shiftVec ){
	bool SHOWDEBUG = false;
	size_t i       = 0        ,
		   numRows = V.rows() ; 
	Eigen::Vector3d tempVec;
	for( i = 0 ; i < numRows ; i++ ){ 
		tempVec = V.row(i);
		tempVec += shiftVec; 
		V.row(i) = tempVec;
	}
	if( SHOWDEBUG ){  cout << "Shift Vec: " << shiftVec << endl;  }
}

void rotate_directions_inplace( Eigen::MatrixXd& dirs , const Eigen::Quaterniond& rotQuat ){
    size_t i       = 0           ,
		   numRows = dirs.rows() ; 
	Eigen::Vector3d tempVec;
	for( i = 0 ; i < numRows ; i++ ){ 
		tempVec     = dirs.row(i);
		tempVec     = rotQuat * tempVec; 
		dirs.row(i) = tempVec;
	}
}

Eigen::Vector3d Eig_vec3d_round_zero( const Eigen::Vector3d& vec ){
	size_t i = 0;
	Eigen::Vector3d rtnVec;
	for( i = 0 ; i < 3 ; i++ ){ if( abs( vec(i) ) < EPSILON ){ rtnVec(i) = 0.0d; }else{ rtnVec(i) = vec(i); } }
	return rtnVec;
}

// Test if Eigen 3D vectors are "about equal"
bool vec3d_eq( const Eigen::Vector3d& v1 , const Eigen::Vector3d& v2 ){ return ( v1 - v2 ).norm() <= EPSILON; }

Eigen::Vector3d vec_avg( const Eigen::MatrixXd& vertices ){
	// Return the average R3 vector of a matrix in which each row is an R3 vector
	size_t len = vertices.rows();
	Eigen::Vector3d accumVec{ 0.0 , 0.0 , 0.0 };
	Eigen::Vector3d rowVec;
	for( size_t i = 0 ; i < len ; i++ ){
		rowVec = vertices.row(i);
		accumVec += rowVec;
	}
	return accumVec / ( (double)len );
}

Eigen::Vector3d err_vec3(){  // Return a 3D vec populated with NaN
	return Eigen::Vector3d( nan("") , nan("") , nan("") );  
} 

Eigen::Quaterniond err_quat(){  // Return a quaternion populated with NaN
	return Eigen::Quaterniond( nan("") , nan("") , nan("") , nan("") );  
} 

Eigen::MatrixXd err_matx(){  // Return a matrix populated with NaN
	Eigen::MatrixXd rtnMatx = Eigen::MatrixXd::Zero( 1 , 3 );
	rtnMatx << nan("") , nan("") , nan("");
	return rtnMatx;  
} 

Eigen::Vector3d origin_vec(){  return Eigen::Vector3d(0,0,0);  } // Return a vector that represents no translation

Eigen::Quaterniond no_turn_quat(){  return Eigen::Quaterniond( 1.0 , 0.0 , 0.0 , 0.0 );  } // Return a quaternion that represents no rotation

// Test if the error vector was returned
bool is_err( const Eigen::Vector3d& query ){  return isnan( query(0) ) || isnan( query(1) ) || isnan( query(2) );  }

// Test if the error quaternion was returned
bool is_err( const Eigen::Quaterniond& query ){  return isnan( query.w() ) || isnan( query.x() ) || isnan( query.y() ) || isnan( query.z() );  }

// Test if the error pose was returned
bool is_err( const Pose_ASP& query ){  return (  is_err( query.position )  &&  is_err( query.orientation )  );  }

// Test if the error matrix     was returned
bool is_err( const Eigen::MatrixXd& query ){
	size_t len_i = query.rows() ,
		   len_j = query.cols() ;
	for( size_t i = 0 ;  i < len_i ; i++ ){
		for( size_t j = 0 ; j < len_j ; j++ ){
			if( !isnan( query( i , j ) ) ){  return false;  }
		}
	}
	return true;
}


string vertices_to_string( Eigen::MatrixXd& V ){
	// Return a string representing a list of vertices that is 
	stringstream rtnStr;
	size_t len = V.rows();
	for( size_t i = 0 ; i < len ; i++ ){
		rtnStr << V( i , 0 ) << " , " << V( i , 1 ) << " , " << V( i , 2 ) << endl;
	}
	return rtnStr.str();
}

string facets_to_string( Eigen::MatrixXi& F ){
	// Return a string representing a list of vertices that is 
	stringstream rtnStr;
	size_t len = F.rows();
	for( size_t i = 0 ; i < len ; i++ ){
		rtnStr << F( i , 0 ) << " , " << F( i , 1 ) << " , " << F( i , 2 ) << endl;
	}
	return rtnStr.str();
}

Pose_ASP err_pose(){ // Return a 3D pose populated with NaN
	Pose_ASP rtnStruct;
	rtnStruct.position    = err_vec3();
	rtnStruct.orientation = err_quat();
	return rtnStruct;
}

bool perp_w_margin( Eigen::Vector3d v1 , Eigen::Vector3d v2 , double CRIT_ANG ){ 
	return abs( angle_between( v1 , v2 ) - ( M_PI / 2.0 ) ) <= CRIT_ANG;
}

// = Transformation / Pose =

Eigen::Matrix3d basis_vecs_to_rot_matx( const Eigen::Vector3d& xBasis_B , 
										const Eigen::Vector3d& yBasis_B , 
										const Eigen::Vector3d& zBasis_B ){
	Eigen::Matrix3d rtnMatx = Eigen::MatrixXd( 3 , 3 );
	rtnMatx <<  xBasis_B(0) , yBasis_B(0) , zBasis_B(0) ,
			    xBasis_B(1) , yBasis_B(1) , zBasis_B(1) ,
			    xBasis_B(2) , yBasis_B(2) , zBasis_B(2) ;
	return rtnMatx;
}

Eigen::Quaterniond basis_vecs_to_quat( const Eigen::Vector3d& xBasis_B , 
									   const Eigen::Vector3d& yBasis_B , 
									   const Eigen::Vector3d& zBasis_B ){
	return Eigen::Quaterniond( basis_vecs_to_rot_matx( xBasis_B , yBasis_B , zBasis_B ) );
}



/** basis_change 
 * @brief Express 'vec_A' in the A frame in the B frame , given the B frame bases vectors expressed in the A frame
 *
 *  @param vec_A    - Vector to be changed to a new basis , R3
 *  @param xBasis_B - X basis vector for frame B
 *  @param yBasis_B - Y basis vector for frame B
 *  @param zBasis_B - Z basis vector for frame B
 *  @return Eigen::Vector3d
 */
Eigen::Vector3d basis_change( Eigen::Vector3d& vec_A , 
							  Eigen::Vector3d& xBasis_B , Eigen::Vector3d& yBasis_B , Eigen::Vector3d& zBasis_B ){
	Eigen::Vector3d rtnVec;
	rtnVec << vec_A.dot( xBasis_B ) , vec_A.dot( yBasis_B ) , vec_A.dot( zBasis_B ) ;
	return rtnVec;
}

Eigen::Vector3d basis_change( Eigen::Vector3d& vec_A , Pose_ASP basisPose ){ return basisPose.orientation * vec_A; }

Eigen::Vector3d point_basis_change( Eigen::Vector3d point_A  , Eigen::Vector3d origin_B , 
									Eigen::Vector3d xBasis_B , Eigen::Vector3d yBasis_B , Eigen::Vector3d zBasis_B ){
	Eigen::Vector3d offset = point_A - origin_B;
	return basis_change( offset , xBasis_B , yBasis_B , zBasis_B );
}

Eigen::Vector3d point_basis_change( Eigen::Vector3d point_A , Pose_ASP basisPose ){
	Eigen::Vector3d offset = point_A - basisPose.position;
	return basis_change( offset , basisPose );
}

Eigen::Vector3d vec3d_from_arbitrary_2D_basis( double x , double y , Eigen::Vector3d xBasis , Eigen::Vector3d yBasis ){
	return xBasis * x + yBasis * y; // DO NOT normalize the basis vectors , see below!
}

Eigen::Vector3d transform_point( Eigen::Vector3d point_A , Pose_ASP basisPose ){
	return basisPose.orientation * point_A + basisPose.position;
}

Eigen::Vector3d transform_point( Eigen::Vector3d point_A , 
								 Eigen::Vector3d origin , 
							     Eigen::Vector3d xBasis , Eigen::Vector3d yBasis , Eigen::Vector3d zBasis ){
	return ( xBasis * point_A(0) ) + ( yBasis * point_A(1) ) + ( zBasis * point_A(2) ) + origin;
}

Eigen::Vector3d transform_vector( Eigen::Vector3d vec_A , Pose_ASP basisPose ){
	return basisPose.orientation * vec_A;
}

bool eq( const Eigen::Quaterniond& op1 , const Eigen::Quaterniond& op2 ){
	// Determine if the two quaternions are equal (enough)
	return (  eq( op1.w() , op2.w() )  ) 
	    && (  eq( op1.x() , op2.x() )  ) 
	    && (  eq( op1.y() , op2.y() )  ) 
	    && (  eq( op1.z() , op2.z() )  );
}       

Eigen::Quaterniond RPY_to_Quat( double Roll , double Pitch , double Yaw ){
	// URL , RPY to quaternion: https://stackoverflow.com/a/43192748
	Eigen::Quaterniond q;
	q = Eigen::AngleAxisd( Roll  , Eigen::Vector3d::UnitX() )
	  * Eigen::AngleAxisd( Pitch , Eigen::Vector3d::UnitY() )
	  * Eigen::AngleAxisd( Yaw   , Eigen::Vector3d::UnitZ() );
	return q;
}

Eigen::Vector3d get_CCW_tri_norm( const Eigen::Vector3d& v0 , const Eigen::Vector3d& v1 , const Eigen::Vector3d& v2 ){
	Eigen::Vector3d xBasis = ( v1 - v0 ).normalized();
	Eigen::Vector3d vecB   = ( v2 - v0 ).normalized();
	return xBasis.cross( vecB ).normalized(); // This should already be normalized
}

Eigen::Vector3d get_CCW_tri_norm( const Eigen::MatrixXd& V ){
	Eigen::Vector3d v0 = V.row(0);
	Eigen::Vector3d v1 = V.row(1);
	Eigen::Vector3d v2 = V.row(2);
	return get_CCW_tri_norm( v0 , v1 , v2 );
}

Eigen::MatrixXd N_from_VF( const Eigen::MatrixXd& V , const Eigen::MatrixXi& F ){
	size_t len = F.rows();
	Eigen::MatrixXd allNorms = Eigen::MatrixXd::Zero( len , 3 );
	Eigen::Vector3d v0 , v1 , v2;
	for( size_t i = 0 ; i < len ; i++ ){
		v0 = V.row( F( i , 0 ) );
		v1 = V.row( F( i , 1 ) );
		v2 = V.row( F( i , 2 ) );
		allNorms.row( i ) = get_CCW_tri_norm( v0 , v1 , v2 );
	}
	return allNorms;
}

Eigen::MatrixXd vertex_norms_from_reduced_VF( const Eigen::MatrixXd& V , const Eigen::MatrixXi& F ){
	// Get the vertex normals from a trimesh
	// NOTE: This function assumes that duplicate vertices have been removed and that facets match
	Eigen::MatrixXd N_vertices;
	igl::per_vertex_normals( V , F , 
							 //~ igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_AREA , // Pointing at uneven angles
							 //~ igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_UNIFORM , // Even more wonky
							 igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_ANGLE , // Good results , No inward-pointing norms
							 N_vertices );
	return N_vertices;
}

Eigen::MatrixXd AABB( const Eigen::MatrixXd& V ){
	// Return the minimum and maximum corners of an AA Bounding Box of arbitrary dimension , Return size 2 x M
	size_t numCols = V.cols() , 
		   numRows = V.rows() ;
	Eigen::MatrixXd corners = Eigen::MatrixXd::Zero( 2 , numCols );
	corners.row(0) = V.row(0);  corners.row(1) = V.row(0); // Init corners for a proper comparision
	for( size_t i = 0 ; i < numRows ; i++ ){
		for( size_t j = 0 ; j < numCols ; j++ ){
			corners( 0 , j ) = min( corners( 0 , j ) , V( i , j ) ); // min corner
			corners( 1 , j ) = max( corners( 1 , j ) , V( i , j ) ); // max corner
		}
	}
	return corners;
}

// Return the minimum and maximum corners of an AA Bounding Box of 'mesh'
Eigen::MatrixXd AABB( const TriMeshVFN_ASP& mesh ){  return AABB( mesh.V );  } 

bool is_point_inside_AABB( const Eigen::Vector3d& query , const Eigen::MatrixXd& bbox ){
    for( size_t i = 0 ; i < 3 ; i++ ){
        if( query(i) < bbox( 0 , i ) ) return false;
        if( query(i) > bbox( 1 , i ) ) return false;
    }
    return true;
}

bool do_AABBs_intersect( const Eigen::MatrixXd& bbox1 , const Eigen::MatrixXd& bbox2 ){
    // Return true if 'bbox1' and 'bbox2'
    Eigen::Vector3d crnr1;  Eigen::Vector3d crnr2;
    std::vector<std::vector<size_t>> indicesVec = enumerate_in_base( 3 , 2 );
    // NOTE: This function assumes that 'bbox1' and 'bbox2' are R3
    // 1. For each combination of corner coordinates
    for( size_t d = 0 ; d < 8 ; d++ ){
        // 2. For each dimension , Build the vector
        for( size_t j = 0 ; j < 3 ; j++ ){
            // 3. Fetch corner dim of 'bbox1'
            crnr1(j) = bbox1( indicesVec[d][j] , j );
            // 4. Fetch corner dim of 'bbox2'
            crnr2(j) = bbox2( indicesVec[d][j] , j );
        }
        // 5. Determine if corner 1 is box 2
        if( is_point_inside_AABB( crnr1 , bbox2 ) ){  return true;  }
        // 6. Determine if corner 2 is box 1
        if( is_point_inside_AABB( crnr2 , bbox1 ) ){  return true;  }
    }
    // 7. If we made it this far, no bbox corner lies in the other bbox
    return false;
}

Eigen::MatrixXd arbitrary_BB_parent_frame( const Eigen::MatrixXd& V ,
										   const Eigen::Vector3d& origin , 
										   const Eigen::Vector3d& xBasis , const Eigen::Vector3d& yBasis , const Eigen::Vector3d& zBasis ){
	// Return the corners of a Bounding Box aligned with the given bases , Corners expressed in the lab frame , Bases expressed in lab frame
	// NOTE: This function assumes that 'V' has at least one point
	// NOTE: This function assumes that 'V' has dimensions ( N , 3 ) for N points
	
	// 1. Transform points
	Eigen::MatrixXd Vtrans = V_in_child_frame( V , origin , xBasis , yBasis , zBasis );
	// 2. Take AABB
	Eigen::MatrixXd cornersTrans = AABB( Vtrans );
	// 3. Untransform AABB  &&  Return
	return V_in_parent_frame( cornersTrans , origin , xBasis , yBasis , zBasis );
}

Eigen::MatrixXd box_corners_from_bases_and_extents( const Eigen::Vector3d& origin , 
										            const Eigen::Vector3d& xBasis , const Eigen::Vector3d& yBasis , const Eigen::Vector3d& zBasis ,
										            const std::vector<double>& extents ){
    if( extents.size() == 6 ){
        Eigen::MatrixXd rtnPoints = Eigen::MatrixXd::Zero( 8 , 3 );
        Eigen::Vector3d currPnt;
        size_t /* -- */ counter = 0;
        for( size_t xDex = 0 ; xDex < 2 ; xDex++ ){
            for( size_t yDex = 2 ; yDex < 4 ; yDex++ ){
                for( size_t zDex = 4 ; zDex < 6 ; zDex++ ){
                    currPnt = origin + xBasis*extents[ xDex ] + yBasis*extents[ yDex ] + zBasis*extents[ zDex ];
                    rtnPoints.row( counter ) = currPnt;
                    counter++;
                }
            }
        }
        return rtnPoints;
    }else{  throw std::out_of_range ( "box_corners_from_bases_and_extents , Got an 'extents' vector of improper length " +
                                      to_string( extents.size() ) + ". The correct size is 6."  );   }
}

double extent_in_direction( const Eigen::MatrixXd& V , const Eigen::Vector3d& direction ){
	// Return the span that 'V' covers in the given 'direction'
	size_t numRows = V.rows();
	Eigen::Vector3d currPnt;
	double most    = -BILLION_D , 
		   least   =  BILLION_D ,
		   projMag =  0.0       ;
	for( size_t i = 0 ; i < numRows ; i++ ){
		projMag = vec3_project_mag( currPnt , direction );
		most    = max( most , projMag );
		least   = min( least , projMag );
	}
	return abs( most - least );
}

Eigen::MatrixXd sample_from_AABB( size_t N , const Eigen::MatrixXd& aabb ){
	// Return 'N' uniform, random samples from AABB
	Eigen::Vector3d crnr1 = aabb.row(0);
	Eigen::Vector3d crnr2 = aabb.row(1);
	Eigen::MatrixXd rtnMatx = Eigen::MatrixXd::Zero( N , 3 );
	for( size_t i = 0 ; i < N ; i++ ){
		rtnMatx.row(i) = vec3d_rand_corners( crnr1 , crnr2 );
	}
	return rtnMatx;
}

Eigen::Vector3d sample_from_AABB( const Eigen::MatrixXd& aabb ){
	// Return a uniform, random samples from AABB
	Eigen::Vector3d crnr1 = aabb.row(0);
	Eigen::Vector3d crnr2 = aabb.row(1);
	return vec3d_rand_corners( crnr1 , crnr2 );
} 

Eigen::MatrixXd span_from_AABB( Eigen::MatrixXd aabb ){
	// Return the extent of each dimension of an AABB as a row vector
	size_t numCols = aabb.cols();
	Eigen::MatrixXd span = Eigen::MatrixXd::Zero( 1 , numCols );
	for( size_t j = 0 ; j < numCols ; j++ ){ span( 0 , j ) = abs( aabb( 1 , j ) - aabb( 0 , j ) ); }
	return span;
}

//~ def tri_normal( p0 , p1 , p2 ):
Eigen::Vector3d tri_normal( Eigen::Vector3d p0 , Eigen::Vector3d p1 , Eigen::Vector3d p2 ){
    //~ """ Return the unit normal vector for a triangle with points specified in CCW order """
    //~ vec1 = np.subtract( p1 , p0 )
    Eigen::Vector3d vec1 = p1 - p0;
    //~ vec2 = np.subtract( p2 , p0 )
    Eigen::Vector3d vec2 = p2 - p0;
    //~ return vec_unit( np.cross( vec1 , vec2 ) )
    return vec1.cross( vec2 ).normalized();
}

Eigen::Vector3d tri_normal( Eigen::MatrixXd CCWverts ){
	//~ """ Return the unit normal vector for a triangle with points specified in CCW order """
	Eigen::Vector3d p0 = CCWverts.row(0);
	Eigen::Vector3d p1 = CCWverts.row(1);
	Eigen::Vector3d p2 = CCWverts.row(2);
	return tri_normal( p0 , p1 , p2 );
}

double tri_area( Eigen::Vector3d p0 , Eigen::Vector3d p1 , Eigen::Vector3d p2 ){ 
	// Return area of CCW triangle in R3
	Eigen::Vector3d v1 = p1 - p0;
	Eigen::Vector3d v2 = p2 - p0;
	return ( v1.cross( v2 ).norm() ) / 2.0; // Length of resultant of cross product is equal area of parallelogram spanned by vectors
}

double tri_area( Eigen::MatrixXd CCWverts ){ 
	// Return area of CCW triangle in R3
	Eigen::Vector3d p0 = CCWverts.row(0);
	Eigen::Vector3d p1 = CCWverts.row(1);
	Eigen::Vector3d p2 = CCWverts.row(2);
	return tri_area( p0 , p1 , p2 );
}

bool are_those_triangles_adjacent( Eigen::MatrixXd triA , Eigen::MatrixXd triB , double CRIT_ANG , double CRIT_DST ){ 
	// Return true if close
	// NOTE: Traingle A is considered to be the projection plane
	// 1. Get the normals
	Eigen::MatrixXd normA = tri_normal( triA );
	Eigen::MatrixXd normB = tri_normal( triB );
	// 2. Get the angle between the normals and check criterion
	double angle = angle_between( normA , normB );
	if( abs( angle - M_PI ) <= CRIT_ANG ){
		// 3. Get perpendicular distance of Tri B point 1 to Tri A and check crtierion
		Eigen::Vector3d planePnt = triA.row(0);
		Eigen::Vector3d queryPnt = triB.row(0);
		double separation = abs( dist_to_plane( 
			normA ,   // plane normal
			planePnt , // plane point
			queryPnt // point to measure
		) );
		// 4. If both criteria met , Check for collision / overlap
		if( separation <= CRIT_DST ){
			Eigen::Vector3d p0A = triA.row(0);
			Eigen::Vector3d p1A = triA.row(1);
			// A. Get X Basis , Tri A
			Eigen::Vector3d xBasisA = ( p1A - p0A ).normalized();
			// C. Project all points to Tri A plane
			Eigen::MatrixXd triAflat = verts3d_proj_to_plane_2D( triA , planePnt , normA , xBasisA );
			Eigen::MatrixXd triBflat = verts3d_proj_to_plane_2D( triB , planePnt , normA , xBasisA );
			// D. Check collision
			return polygon_collide_2D( triAflat , triBflat , false , true );
		}else{  return false;  }
	}else{  return false;  }
}

Eigen::Vector3d line_intersect_plane( Eigen::Vector3d rayOrg , Eigen::Vector3d rayDir , 
									  Eigen::Vector3d planePnt , Eigen::Vector3d planeNrm ,
									  bool pntParallel ){
	// URL , Intersection point between line and plane: https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
	// NOTE: Line is defined by a ray lying on line, though this function will return an intersection point on either side of the ray origin
	//       whichever side it occurs
	double d = 0.0;
	// If the ray direction and plane normal are perpendicular, then the ray is parallel to the plane
	if(  eq( rayDir.dot( planeNrm ) , 0.0 )  ){
		// if the line segment between the plane point and the ray origin has no component in the plane normal direction, plane contains ray
		if(  eq( ( planePnt - rayOrg ).dot( planeNrm ) , 0.0 )  ){  
			if( pntParallel ){  return rayOrg;  } // If ray origin as an appropriate stand-in for the intersection of coplanar line
			else{  return err_vec3();  } // else return no-intersection
		} // Return ray origin for sake of convenience
		else{  return err_vec3();  } // else the ray is apart from and parallel to the plane, no intersection to ret
	}else{ // else the ray and plane intersect at exactly one point
		// 1. Calculate the distance along the ray that the intersection occurs
		d = ( ( planePnt - rayOrg ).dot( planeNrm ) ) / ( rayDir.dot( planeNrm ) );
		return rayOrg + rayDir * d;
	}
}

//~ def circle_arc_3D( axis , center , radius , beginMeasureVec , theta , N ):
Eigen::MatrixXd circle_arc_3D( const Eigen::Vector3d& axis , const Eigen::Vector3d& center , 
							   double radius , const Eigen::Vector3d& beginMeasureVec , double theta , size_t N ){
	// Return points on a circular arc about 'axis' at 'radius' , beginning at 'beginMeasureVec' and turning 'theta' through 'N' points
	
	bool SHOWDEBUG = false;
	
	if( SHOWDEBUG ){  cout << "Allocate vars ..." << endl;  }
	
	Eigen::MatrixXd rtnList = Eigen::MatrixXd::Zero( N , 3 );
    //~ thetaList = np.linspace( 0 , theta , N )
    std::vector<double> thetaList = linspace( 0.0d , theta , N );
    //~ k         = vec_unit( axis )
    Eigen::Vector3d k = axis.normalized();
    //~ # 1. Project the theta = 0 vector onto the circle plane
    //~ measrVec = vec_unit( vec_proj_to_plane( beginMeasureVec , axis ) )
    Eigen::Vector3d measrVec = vec_proj_to_plane( beginMeasureVec , axis ).normalized();
    
    if( SHOWDEBUG ){  cout << "About to iterate ..." << endl;  }
    
    //~ # 2. For each theta
    //~ for theta_i in thetaList:
    for( size_t i = 0 ; i < N ; i++ ){
		
		if( SHOWDEBUG ){  cout << "Get rotation matrix ..." << endl;  }
		
        //~ # 3. Create a rotation matrix for the rotation
        //~ R = rot_matx_ang_axs( theta_i , k  )
        Eigen::AngleAxis<double> rot( thetaList[i] , k );
        Eigen::Matrix3d R = rot.toRotationMatrix();
        
        if( SHOWDEBUG ){  cout << "Apply rotation matrix" << endl << R << endl;  }
        
        //~ # 4. Apply rotation to the measure vector && 5. Scale the vector to the radius && 6. Add to the center && 7. Append to the list
        //~ rtnList.append( np.add( center , np.multiply( np.dot( R , measrVec ) , radius ) ) )
        rtnList.row(i) = center + ( R * measrVec ) * radius;
	}
    //~ # 8. Return
    return rtnList;
}

Eigen::MatrixXd linspace_pts_3D( Eigen::Vector3d bgn , Eigen::Vector3d end , size_t N ){
	// Return a sequence of 'N' 3D points from 'bgn' to 'end' (inclusive)
	Eigen::MatrixXd rtnPts = Eigen::MatrixXd::Zero( N , 3 );
	
	Eigen::Vector3d diff = end - bgn;
	Eigen::Vector3d dirL = diff.normalized();
	double totalDist = diff.norm();
	std::vector<double> distList = linspace( 0.0d , totalDist , N );
	Eigen::Vector3d point;
	
	for( size_t i = 0 ; i < N ; i++ ){
		point = bgn + dirL * distList[i];
		rtnPts.row(i) = point;
	}
	
	return rtnPts;
}

Eigen::Vector3d closest_point_to( Eigen::MatrixXd points , Eigen::Vector3d queryPnt ){
	Eigen::Vector3d compare = points.row(0);
	Eigen::Vector3d closest = compare;
	double dist     = ( compare - queryPnt ).squaredNorm();
	double shortest = dist;
	size_t len     = points.rows();
	for( size_t i = 1 ; i < len ; i++ ){
		compare = points.row(i);
		dist = ( compare - queryPnt ).squaredNorm();
		if( dist < shortest ){
			shortest = dist;
			closest = compare;
		}
	}
	return closest;
}	

Eigen::Vector3d farthest_point_from( Eigen::MatrixXd points , Eigen::Vector3d queryPnt ){
	Eigen::Vector3d compare = points.row(0);
	Eigen::Vector3d furthest = compare;
	double dist     = ( compare - queryPnt ).squaredNorm();
	double longest = dist;
	size_t len     = points.rows();
	for( size_t i = 1 ; i < len ; i++ ){
		compare = points.row(i);
		dist = ( compare - queryPnt ).squaredNorm();
		if( dist > longest ){
			longest  = dist;
			furthest = compare;
		}
	}
	return furthest;
}	

IndexDbblResult closest_point_to_sq( Eigen::MatrixXd points , Eigen::Vector3d queryPnt ){
	// Return the index of the point that is the closest squared distance to 'queryPnt', as well as the squared distance , linear search
	Eigen::Vector3d compare = points.row(0);
	double dist  = 0.0;
	IndexDbblResult rtnStruct{ 0 , ( compare - queryPnt ).squaredNorm() };
	size_t len   = points.rows();
	for( size_t i = 1 ; i < len ; i++ ){
		compare = points.row(i);
		dist = ( compare - queryPnt ).squaredNorm();
		if( dist < rtnStruct.measure ){
			rtnStruct.index = i;
			rtnStruct.measure = dist;
		}
	}
	return rtnStruct;
}

IndexDbblResult farthest_point_from_sq( Eigen::MatrixXd points , Eigen::Vector3d queryPnt ){
	// Return the index of the point that is the closest squared distance to 'queryPnt', as well as the squared distance , linear search
	Eigen::Vector3d compare = points.row(0);
	double dist  = 0.0;
	IndexDbblResult rtnStruct{ 0 , ( compare - queryPnt ).squaredNorm() };
	size_t len   = points.rows();
	for( size_t i = 1 ; i < len ; i++ ){
		compare = points.row(i);
		dist = ( compare - queryPnt ).squaredNorm();
		if( dist > rtnStruct.measure ){
			rtnStruct.index = i;
			rtnStruct.measure = dist;
		}
	}
	return rtnStruct;
}

IndexDbblResult closest_point_to_sq( std::vector<double> points , double queryPnt ){
	double compare = points[0];
	double dist    = pow( abs( compare - queryPnt ) , 2 );
	size_t len     = points.size();
	IndexDbblResult rtnStruct{ 0 , pow( abs( compare - queryPnt ) , 2 ) };
	for( size_t i = 1 ; i < len ; i++ ){
		compare = points[i];
		dist = pow( abs( compare - queryPnt ) , 2 );
		if( dist < rtnStruct.measure ){
			rtnStruct.index = i;
			rtnStruct.measure = dist;
		}
	}
	return rtnStruct;
}

Eigen::Quaterniond unif_rand_quat(){
	// Generate a uniformly-distributed random unit quaternion
	double s      = rand_dbbl()            ,
		   sig1   = sqrt( 1 - s )          ,
		   sig2   = sqrt( s )              ,
		   theta1 = 2 * M_PI * rand_dbbl() , 
		   theta2 = 2 * M_PI * rand_dbbl() ;
	return Eigen::Quaterniond(
		cos( theta2 ) * sig2 , // w
		sin( theta1 ) * sig1 , // x
		cos( theta1 ) * sig1 , // y
		sin( theta2 ) * sig2 // _ z
	);
}

// == Direction Counting ==

// Copy the rows of 'original' that represent unique directions
Eigen::MatrixXd uniqify_directions( const Eigen::MatrixXd& directions , double CRIT_ANG ){
	size_t numDir    = directions.rows();
	Eigen::Vector3d currDir;
	Eigen::MatrixXd uniqDirs;
	
	auto contains_dir = [&]( Eigen::Vector3d direction ){
		size_t len      = uniqDirs.rows();
		double unqAngle = 0.0;
		bool   present  = false;
		Eigen::Vector3d dir_i;
		for( size_t k = 0 ; k < len ; k++ ){
			dir_i = uniqDirs.row(k);
			unqAngle = angle_between( dir_i , direction );
			if(  eq( unqAngle , 0.0 , CRIT_ANG )  ){
				present = true;
				break;
			}
		}
		return present;
	};
	
	for( size_t i = 0 ; i < numDir ; i++ ){
		currDir = directions.row(i);
		if( !contains_dir( currDir ) ){
			uniqDirs = copy_V_plus_row( uniqDirs , currDir );
		}
	}
	return uniqDirs;
}

size_t count_unique_dirs( const Eigen::MatrixXd& directions , double CRIT_ANG ){
	return uniqify_directions( directions , CRIT_ANG ).rows();
}

// Return a row list of all of the unique directions that opposing vectors are colinear with
Eigen::MatrixXd get_unopposed_dirs( const Eigen::MatrixXd& directions , double CRIT_ANG ){
	// Count all of the directions that do not have an opposing direction
	// NOTE: This function does not consider uniqueness , There may be repeat, unopposed directions
	size_t numDir      = directions.rows() ;
	bool   hasOpposing = false;
	double angle       = 0.0;
	Eigen::Vector3d dir_i;
	Eigen::Vector3d dir_j;
	Eigen::MatrixXd unopposed;
	for( size_t i = 0 ; i < numDir ; i++ ){
		hasOpposing = false;
		dir_i = directions.row(i);
		for( size_t j = 0 ; j < numDir ; j++ ){
			if( i != j ){
				dir_j = directions.row(j);
				angle = angle_between( dir_i , dir_j );
				if( abs( angle - M_PI ) < CRIT_ANG ){  hasOpposing = true;  }
			}
		}
		if( !hasOpposing ){  unopposed = copy_V_plus_row( unopposed , dir_i );  }
	}
	return unopposed;
}

size_t count_unopposed( const Eigen::MatrixXd& directions , double CRIT_ANG ){
	// Count all of the directions that do not have an opposing direction
	return get_unopposed_dirs( directions , CRIT_ANG ).rows();
}


Eigen::MatrixXd get_opposing_dirs( const Eigen::MatrixXd& directions , double CRIT_ANG ){
	// Return a row list of all of the unique directions that opposing vectors are colinear with
	size_t numDir      = directions.rows() ;
	bool   hasOpposing = false;
	double angle       = 0.0;
	Eigen::Vector3d dir_i;
	Eigen::Vector3d dir_j;
	Eigen::MatrixXd opposingDirs;
	
	// LAMBDA: [&] Capture all named variables by reference
	// Type defaults to void if there is no return statement
	auto hasColinearDir = [&]( Eigen::Vector3d direction ){
		bool   hasColin = false;
		size_t len      = opposingDirs.rows();
		double coAngle  = 0.0;
		Eigen::Vector3d currDir;
		for( size_t k = 0 ; k < len ; k++ ){
			currDir = opposingDirs.row(k);
			coAngle = angle_between( direction , currDir );
			
			// DANGER : CRITERION CHANGED
			if(  eq( coAngle , 0.0 , CRIT_ANG )  ||  eq( coAngle , M_PI , CRIT_ANG )  ){
			//~ if(  eq( coAngle , M_PI , CRIT_ANG )  ){
			
				hasColin = true;
				break;
			}
		}
		return hasColin;
	};
	for( size_t i = 0 ; i < numDir ; i++ ){
		hasOpposing = false;
		for( size_t j = 0 ; j < numDir ; j++ ){
			if( i != j ){
				dir_i = directions.row(i);
				dir_j = directions.row(j);
				angle = angle_between( dir_i , dir_j );
				if( eq( angle , M_PI , CRIT_ANG ) ){
					if( !hasColinearDir( dir_i ) ){
						opposingDirs = copy_V_plus_row( opposingDirs , dir_i );
						break;
					}
				}
			}
		}
	}
	return uniqify_directions( opposingDirs );
}

size_t count_opposing_dirs( const Eigen::MatrixXd& directions , double CRIT_ANG ){
	// Count all of the directions that do not have an opposing direction
	return get_opposing_dirs( directions , CRIT_ANG ).rows();
}



// __ End Counting __

Eigen::Vector3d get_any_perpendicular( const Eigen::Vector3d& query , double CRIT_ANG ){
	// Get any unit vector that is perpendicular to 'query'
	Eigen::Vector3d op = vec3d_random();
	while(  eq( angle_between( op , query ) , 0.0 , CRIT_ANG )  ){  op = vec3d_random();  }
	return op.cross( query ).normalized();
}

Eigen::Vector3d get_any_perpendicular( const Eigen::Vector3d& query , const Eigen::Vector3d& perpSuggest , double CRIT_ANG ){
    // Return the unit vector perpendicular to 'query' that is closest to 'perpSuggest'
    Eigen::Vector3d op = perpSuggest;
    while(  eq( angle_between( op , query ) , 0.0 , CRIT_ANG )  ){  op = vec3d_random();  } // If the suggestion was bad, random choice
    return query.cross( perpSuggest.cross( query ) ).normalized();
}

Eigen::MatrixXd get_any_orthogBasis_for_X( const Eigen::Vector3d& Xquery , double CRIT_ANG ){
	Eigen::MatrixXd bases  = Eigen::MatrixXd::Zero( 3 , 3 );
	Eigen::Vector3d xBasis = Xquery.normalized();
	Eigen::Vector3d yBasis = get_any_perpendicular( Xquery , CRIT_ANG ); // Returns a normalized vector
	Eigen::Vector3d zBasis = xBasis.cross( yBasis ).normalized();
	bases.row(0) = xBasis;
	bases.row(1) = yBasis;
	bases.row(2) = zBasis;
	return bases;
}

Eigen::MatrixXd get_any_orthogBasis_for_Z( const Eigen::Vector3d& Zquery , double CRIT_ANG ){
	Eigen::MatrixXd bases  = Eigen::MatrixXd::Zero( 3 , 3 );
	Eigen::Vector3d zBasis = Zquery.normalized();
	Eigen::Vector3d yBasis = get_any_perpendicular( Zquery , CRIT_ANG ); // Returns a normalized vector
	Eigen::Vector3d xBasis = yBasis.cross( zBasis ).normalized();
	bases.row(0) = xBasis;
	bases.row(1) = yBasis;
	bases.row(2) = zBasis;
	return bases;
}

Eigen::MatrixXd get_any_orthogBasis_for_Z( const Eigen::Vector3d& Zquery , const Eigen::Vector3d& Xsuggest , double CRIT_ANG ){
	Eigen::MatrixXd bases  = Eigen::MatrixXd::Zero( 3 , 3 );
	Eigen::Vector3d zBasis = Zquery.normalized();
	Eigen::Vector3d xBasis = get_any_perpendicular( Zquery , Xsuggest , CRIT_ANG ); // Returns a normalized vector
	Eigen::Vector3d yBasis = zBasis.cross( xBasis ).normalized();
	bases.row(0) = xBasis;
	bases.row(1) = yBasis;
	bases.row(2) = zBasis;
	return bases;
}

Eigen::MatrixXd cardinal_directions_R3(){
	// Return positive and negative directions along each principal axis
	Eigen::MatrixXd     cardinalDirs = Eigen::MatrixXd::Zero( 6 , 3 ); // 
	cardinalDirs.row(0) /* ------ */ = Eigen::Vector3d::UnitX() *  1.0;
	cardinalDirs.row(1) /* ------ */ = Eigen::Vector3d::UnitX() * -1.0;
	cardinalDirs.row(2) /* ------ */ = Eigen::Vector3d::UnitY() *  1.0;
	cardinalDirs.row(3) /* ------ */ = Eigen::Vector3d::UnitY() * -1.0;
	cardinalDirs.row(4) /* ------ */ = Eigen::Vector3d::UnitZ() *  1.0;
	cardinalDirs.row(5) /* ------ */ = Eigen::Vector3d::UnitZ() * -1.0;
	return cardinalDirs;
}

// ___ End Geometry ___


// === Mesh Operations ===

Eigen::MatrixXd get_facet_centers( const Eigen::MatrixXd& vertices , const Eigen::MatrixXi& facetIndices ){
	// Return an array of R3 points representing the centers of the facets
	size_t i       = 0 ,
		   numRows = facetIndices.rows();
	
	Eigen::MatrixXd rtnMatx = Eigen::MatrixXd::Zero( numRows , 3 );
	
	for( i = 0 ; i < numRows ; i++ ){ // For each of the facets , Compute the vector mean of the facet vertices
		rtnMatx.row( i ) = ( vertices.row( facetIndices( i , 0 ) ) + 
							 vertices.row( facetIndices( i , 1 ) ) + 
							 vertices.row( facetIndices( i , 2 ) ) ) / 3.0;
	}
	return rtnMatx;
}

Eigen::MatrixXi get_adjacency_matx( const Eigen::MatrixXi& facetIndices ){
	// Construct an adjacency matrix for all facets , NOTE: Adjacency is defined by sharing a side , sharing one vertex is not adjacent
	size_t N          = facetIndices.rows() , // Number of facets in the mesh 
		   i          = 0                   , // ----> Counting vars
		   j          = 0                   , //    /
		   k          = 0                   , //   /
		   m          = 0                   , // _/
		   dim        = 3                   , // Number of dimensions
		   shareCount = 0                   , // Number of shared vertices
		   nghbrCount = 0                   ; // Number of 
	Eigen::MatrixXi rtnMatx = Eigen::MatrixXi::Zero( N , N );
	
	for( i = 0 ; i < N - 1 ; i++ ){
		nghbrCount = 0; // In this formulation there can only be 3 neighbors
		for( j = i + 1 ; j < N ; j++ ){
			shareCount = 0;
			for( k = 0 ; k < dim ; k++ ){
				for( m = 0 ; m < dim ; m++ ){
					if( facetIndices( i , k ) == facetIndices( j , m ) ) shareCount++; // If indices are the same , faces share vertex
				}
				if( shareCount > 1 ) break; // If the facets share two vertices , they are adjacent , stop searching
			}
			if( shareCount > 1 ){ // If the facets are adjacent , increment neighbor count and mark adjacency in the matrix
				nghbrCount++;
				rtnMatx( i , j ) = 1; // i is adjacent to j , which means that
				rtnMatx( j , i ) = 1; // j is adjacent to i   also
			}
			if( nghbrCount > 2 ) break; // If there are 3 neighbors , we are at the max , stop searching
		}
	}		
	return rtnMatx;
}

std::vector<std::vector<int>> get_adjacency_list( const Eigen::MatrixXi& facetIndices ){
	// On a closed surface that is properly formed , all facets should always have 3 neighbors , but this assumption is not made
	std::vector<std::vector<int>> rtnStruct;
	size_t i = 0                   ,
		   j = 0                   , 
		   N = facetIndices.rows() ;
	for( i = 0 ; i < N ; i++ ){
		std::vector<int> temp;
		rtnStruct.push_back( temp );
	}
	Eigen::MatrixXi matx = get_adjacency_matx( facetIndices );
	for( i = 0 ; i < N - 1 ; i++ ){
		for( j = i + 1 ; j < N ; j++ ){
			if( matx( i , j ) == 1 ){ 
				rtnStruct[i].push_back( j );
				rtnStruct[j].push_back( i ); 
			}
		}
	}
	return rtnStruct;
}

std::vector<std::vector<int>> facet_adjacency_list_ordered( const Eigen::MatrixXi& F ){
    int indices_RH[3][2] = { { 0 , 1 } , { 1 , 2 } , { 2 , 0 } }; // Right-hand indices
    int indices_LH[3][2] = { { 1 , 0 } , { 2 , 1 } , { 0 , 2 } }; // Left-hand  indices , Neighbor will see vertices of the border segment in reverse order
    int neighbors[3][2];
    int neighbor[2];
    int neighborNeighbor[2];
    //~  = [ [ None , None , None ] for i in xrange( N ) ] # 
    std::vector<std::vector<int>> neighborList;
    int i       = 0        ,
        j       = 0        ,
        k       = 0        ,
        N       = F.rows() ,
        nDex    = 0        ,
        nnDex   = 0        ,
        pairDex = 0        ; // Number of facets;
        
    // Ordered list of neighbors ( neighbor pointers )
    for( i = 0 ; i < N ; i++ ){
		std::vector<int> temp;
		for( int j = 0 ; j < 3 ; j++ ){ temp.push_back( -1 ); }
		neighborList.push_back( temp );
	}
    
    bool foundPtr = false;
    bool match    = true;
    
    //~ for i in xrange( N - 1 ):
    for( i = 0 ; i < N - 1 ; i++ ){
        //~ # Copy right-hand edges for each neighbor position that has not yet been associated with a neighbor
        //~ neighbors = [ indices_RH[ checkDex ] if neighborList[i][ checkDex ] == None else None for checkDex in xrange(3) ] # Avoid repeat search
        for( j = 0 ; j < 3 ; j++ ){
			if( neighborList[i][j] > -1 ) foundPtr = true; else foundPtr = false;
			for( k = 0 ; k < 2 ; k++ ){
				if( !foundPtr ) // If there is not a neighbor at this position , then we will search at this border
					neighbors[j][k] = indices_RH[j][k];
				else // else neighbor found in this direction , mark with invalid index
					neighbors[j][k] = -1;
			}
		}

        for( j = i + 1 ; j < N ; j++ ){ // For each unique pairing of facets ( i , j )
            //~ for nDex , neighbor in enumerate( neighbors ):
            for( nDex = 0 ; nDex < 3 ; nDex++ ){
				for( k = 0 ; k < 2 ; k++){ neighbor[k] = neighbors[nDex][k]; }
                //~ if neighbor != None: # If we have not yet located the neighbor for this edge , Check needed to avoid repeats , see above
                if( neighbor[0] != -1 ){
                    //~ for nnDex , neighborNeighbor in enumerate( indices_LH ):
                    for( nnDex = 0 ; nnDex < 3 ; nnDex++ ){
						for( k = 0 ; k < 2 ; k++){ neighborNeighbor[k] = indices_LH[nnDex][k]; }
                        //~ # nDex : This facet's edge  ,  nnDex : Candidate neighbor edge
                        //~ if neighborList[ j ][ nnDex ] == None:
                        if( neighborList[ j ][ nnDex ] == -1 ){
                            match = true;
                            //~ for pairDex in xrange( 2 ): # 
                            for( pairDex = 0 ; pairDex < 2 ; pairDex++ ){ // For each of the points that form the candidate border
                                // Unless this edge has two vertices that are in reverse order of the other facet edge , 
                                //  this edge is not the border between the two
                                if( F( i , neighbor[ pairDex ] ) != F( j , neighborNeighbor[ pairDex ] ) )
                                    match = false;
							}
                            if( match ){
                                neighborList[ i ][ nDex  ] = j; // Mark the located neighbor
                                for( k = 0 ; k < 2 ; k++){ neighbors[ nDex ][k] = -1; } // Mark this neighbor as found
                                neighborList[ j ][ nnDex ] = i; // This facet is the neighbor's neighbor at its identified border
                            } // else no match , no action , continue
                        //~ # else the neighbor already has a match for this neighbor-nerighbor
						}
					}
				} //~ # else 'neighbor' is Null , we found this neighbor already and there is no action
			}
		}
	}
    return neighborList;
}

TriMeshVFN_ASP remove_duplicate_vertices_from_VF( const Eigen::MatrixXd& OV , const Eigen::MatrixXi& OF ){
	Eigen::MatrixXd V ; 
	Eigen::MatrixXi F , SVI , SVJ;
	Eigen::MatrixXd N;
	TriMeshVFN_ASP rtnStruct;
	igl::remove_duplicate_vertices( OV , OF , 0.0 , V , SVI , SVJ , F );
	rtnStruct.V = V;
	rtnStruct.F = F;
	igl::per_face_normals( V , F , N ); // Compute per-face normals
	rtnStruct.N = N; 
	return rtnStruct;
}

std::vector<std::vector<int>> facet_adjacency_list_ordered_dense( const Eigen::MatrixXd& V , const Eigen::MatrixXi& F ){
	TriMeshVFN_ASP reduced = remove_duplicate_vertices_from_VF( V , F );
	return facet_adjacency_list_ordered( reduced.F );
}

std::list< Segment2D_ASP > get_outside_CCW_poly_from_mesh_subset( const Eigen::MatrixXd& V2D , const Eigen::MatrixXi& F , 
																  const std::vector<std::vector<int>>& orderedNeighborsF , 
																  const std::vector<int>& facetSublist ){
	// Get the border segments from a set of ADJACENT triangles that are part of a larger trimesh
	int i         = 0                        , 
	    numFacets = orderedNeighborsF.size() ,
	    j         = 0                        ,
	    numNghbrs = 3                        ,
	    neighbor  = 0                        ,
	    k         = 0                        ,
	    numTwins  = 2                        ,
	    borderDex = 0                        ;
    int indices_RH[3][2] = { { 0 , 1 } , { 1 , 2 } , { 2 , 0 } }; // Right-hand indices
    int pairDices[2];
    std::list< Segment2D_ASP > borderSegments; // List of segments to return
    std::set<int> regionSet; // # Set of all the facets in the region under consideration
    for( size_t f = 0 ; f < facetSublist.size() ; f++ ){  regionSet.emplace( facetSublist[f] );  }
    
    //~ cout << "DEBUG: " << "There are " << V2D.rows() << " vertices in the array." << endl;
    //~ cout << "DEBUG: " << "The highest facet index is ________ " << F.maxCoeff() << endl;
    //~ cout << "DEBUG: " << "The highest index in the sublist is " << max_num_in_vec( facetSublist ) << endl;
    //~ cout << "DEBUG: " << "Sublist ___________________________ " << facetSublist << endl;
    
    //~ # 1. For each facet
    //~ sep( "SUBSTEP 1 INIT..." , 1 , '!' );
    for( i = 0 ; i < numFacets ; i++ ){
    //~ for i , neighborList in enumerate( orderedNeighborsF ):
		std::vector<int> neighborList = orderedNeighborsF[i];
        //~ # 2. Check that the facet is in the sublist under consideration
        //~ sep( "SUBSTEP 2 INIT..." , 1 , '!' );
        //~ cout << "DEBUG, " << "Neighbors for facet " << i << " are " << neighborList << endl;
        //~ cout << "DEBUG, " << "is "<< i << " in regionSet?: " << regionSet << endl;
        if( is_arg_in_set( i , regionSet ) ){
			//~ cout << "DEBUG: " << "YES!" << endl;
            //~ # 4. For each neighbor , check ( if the neighbor is unoccupied || if the neighbor DNE in the sublist )
            //~ sep( "SUBSTEP 4 INIT..." , 1 , '!' );
            for( j = 0 ; j < numNghbrs ; j++ ){
            //~ for j , neighbor in enumerate( neighborList ):
				neighbor = neighborList[j];
                //~ if ( neighbor == None ) or ( neighbor not in regionSet ):
                if( ( neighbor == -1 ) || ( !is_arg_in_set( neighbor , regionSet ) ) ){
                    //~ # 5. If the above is met , then this neighbor position is an outside border
                    //~ # 6. Fetch a right-hand ordered ( CCW ) segment that represents this border , and append it to the list to be returned
                    //~ pairDices = indices_RH[ j ]
                    for( k = 0 ; k < numTwins ; k++ ){ pairDices[k] = indices_RH[j][k]; }
                    //~ sep( "SUBSTEP 6 CLEAR" , 1 , '!' );
                    //~ segment = []
                    Segment2D_ASP segment;
                    //~ For each of the vertices that make up the border
                    //~ Fetch the element of V that corresponds to the end of the segment that corresponds to the border segment
                    segment.pnt1 = V2D.row( F( i , pairDices[0] ) );
                    segment.pnt2 = V2D.row( F( i , pairDices[1] ) );
                    // Add the segment to the list
                    borderSegments.push_back( segment );
				}
			}
		} else { 
			//~ cout << "DEBUG: " << "NO!" << endl; 
		}
	}
	//~ sep( "SUBSTEP 1 CLEAR" , 1 , '!' );
    // Now we have a list of segments , Now lay them end to end in CCW fashion , then return
    return order_RH_border_segments( segment_list_to_segment_vector( borderSegments ) );
}

Eigen::MatrixXd get_closed_poly_verts_from_CCW_segments( const std::vector< Segment2D_ASP >& segments ){
	// Return a matrix in which each row is a 2D point of a closed polygon // NOTE: This function assumes 'segments' forms a closed polygon
	size_t len = segments.size();
	Eigen::MatrixXd closedPoly = Eigen::MatrixXd::Zero( len + 1 , 2 );
	closedPoly.row(0) = segments[0].pnt1;
	for( size_t i = 0 ; i < len ; i++ ){
		closedPoly.row(i+1) = segments[i].pnt2;
	}
	return closedPoly;
}

Eigen::MatrixXd get_closed_poly_verts_from_CCW_segments( const std::list< Segment2D_ASP >& segments ){
	return get_closed_poly_verts_from_CCW_segments( segment_list_to_segment_vector( segments ) );
}

Eigen::Vector3d vec_from_pnt_to_plane( const Eigen::Vector3d& queryPnt , const Eigen::Vector3d& planePnt , const Eigen::Vector3d& normal ){
    // Return the vector that points from 'queryPnt' to that point projected on a plane defined be 'planePnt' and 'normal'
    return pnt_proj_to_plane( queryPnt , planePnt , normal ) - queryPnt;
}

Eigen::MatrixXd transform_V( const Eigen::MatrixXd& V , const Eigen::Vector3d& position , const Eigen::Quaterniond& orientation ){
	// Return a matrix in which each row is the corresponding vector expressed as part of the 'position' / 'orientation' frame --> lab frame
	size_t len = V.rows();
	Eigen::MatrixXd Vtrans = Eigen::MatrixXd::Zero( len , 3 );
	Eigen::Vector3d temp;
	for( size_t i = 0 ; i < len ; i++ ){
		temp = V.row(i);
		Vtrans.row(i) = ( orientation * temp ) + position;
	}
	return Vtrans;
}

Eigen::MatrixXd V_in_child_frame( const Eigen::MatrixXd& V , 
							      const Eigen::Vector3d& origin , 
							      const Eigen::Vector3d& xBasis , const Eigen::Vector3d& yBasis , const Eigen::Vector3d& zBasis ){
	// Express V in a frame that is contained in V's current frame
	size_t len = V.rows();
	Eigen::MatrixXd Vtrans = Eigen::MatrixXd::Zero( len , 3 );
	Eigen::Vector3d point;
	for( size_t i = 0 ; i < len ; i++ ){
		point = V.row(i);
		Vtrans.row(i) = point_basis_change( point , origin , xBasis , yBasis , zBasis );
	}
	return Vtrans;
}

Eigen::MatrixXd V_in_parent_frame( const Eigen::MatrixXd& V , 
							       const Eigen::Vector3d& origin , 
							       const Eigen::Vector3d& xBasis , const Eigen::Vector3d& yBasis , const Eigen::Vector3d& zBasis ){
	// Express V in a frame that contains V's current frame
	size_t len = V.rows();
	Eigen::MatrixXd Vtrans = Eigen::MatrixXd::Zero( len , 3 );
	Eigen::Vector3d point;
	for( size_t i = 0 ; i < len ; i++ ){
		point = V.row(i);
		Vtrans.row(i) = transform_point( point , origin , xBasis , yBasis , zBasis );
	}
	return Vtrans;
}

Eigen::MatrixXd transform_Dir( const Eigen::MatrixXd& directions , const Eigen::Quaterniond& orientation ){
	// Return a matrix in which each row is the corresponding vector rotated into a new 'orientation'
	size_t len = directions.rows();
	Eigen::MatrixXd dirTrans = Eigen::MatrixXd::Zero( len , 3 );
	Eigen::Vector3d temp;
	for( size_t i = 0 ; i < len ; i++ ){
		temp = directions.row(i);
		dirTrans.row(i) = ( orientation * temp );
	}
	return dirTrans;
}

// == Mesh Transformations ==

TriMeshVFN_ASP shrink_dense_VFN_along_normals( const TriMeshVFN_ASP& original , double shrinkDistance ){ 
	// Shrink the mesh along normals , Input must be dense
	// NOTE: The resulting mesh will NOT be enclosed
	// NOTE: Intersections between triangles at sharp model edges are likely in the resultant mesh
	//       This has resulted in triangles sticking out and causing collisison
	// NOTE: This function assumes that all of the normals are pointing properly outward
	// NOTE: This function is a hack
	TriMeshVFN_ASP rtnStruct = copy_TriMeshVFN_ASP( original );
	//~ rtnStruct.V = original.V;
	//~ rtnStruct.F = original.F; // Facet indices and triangle normals will remain unchanged by this operation
	//~ rtnStruct.N = original.N;
	
	//~ cout << "DEBUG , " << "Original mesh has " << original.V.rows() << " vertices." << endl;
	//~ cout << "DEBUG , " << "Original mesh has " << original.F.rows() << " facets."   << endl;
	//~ cout << "DEBUG , " << "Original mesh has " << original.N.rows() << " normals."  << endl;
	
	size_t len_F = original.F.rows();
	Eigen::Vector3d triCenter , triNormal , vertex , focalPnt , slideDir , V_ij_xform;
	Eigen::MatrixXd centers = get_facet_centers( original.V , original.F );
	double focalLength = 0.0;
	// 0. For each facet
	for( size_t i = 0 ; i < len_F ; i++ ){
		// 1. Find triangle center
		triCenter = centers.row( i );
		// 2. Fetch normal
		triNormal = original.N.row( i );
		// 3. For each vertex
		for( size_t j = 0 ; j < 3 ; j++ ){
			vertex = original.V.row( original.F( i , j ) ); // Fetch vertex
			// 4. Find the length from the triangle center to the vertex --> focal length
			focalLength = ( vertex - triCenter ).norm();
			// 5. Find point offset from center by magnitude focal length and the direction opposite of the triangle normal --> focal point
			focalPnt = triCenter - ( triNormal.normalized() * focalLength );
			// 6. From the vertex to the focal point is the slide direction
			slideDir = ( focalPnt - vertex ).normalized();
			// 7. Move the vertex shrinkDistance * sqrt( 2 ) in the slide direction
			V_ij_xform = vertex + ( slideDir * ( shrinkDistance * sqrt( 2 ) ) );
			rtnStruct.V.row( original.F( i , j ) ) = V_ij_xform;
		}
	}
	return rtnStruct;
}

TriMeshVFN_ASP shrink_dense_VFN_by_Vtx_normals( const TriMeshVFN_ASP& original , double shrinkDistance ){ 
	// Shrink a dense mesh along vertex normals
	// NOTE: This function preserves enclosure, contrary to the above
	// NOTE: This function assumes that all of the normals are pointing properly outward , 2018-05-24: So far observe that they are
	// NOTE: This function returns a reduced mesh, where it is assumed that a dense mesh is passed to this function
	// NOTE: This function is a hack
	Eigen::MatrixXd OV  = original.V , 
					V , N_vertices;
	Eigen::MatrixXi OF  = original.F , 
			        F , SVI , SVJ;
	
	// 1. Reduce mesh
	igl::remove_duplicate_vertices( OV , OF , 0.0 , V , SVI , SVJ , F );
	
	// 2. Retrieve vertex normals
	igl::per_vertex_normals( V , F , 
							 //~ igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_AREA , // Pointing at uneven angles
							 //~ igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_UNIFORM , // Even more wonky
							 igl::PER_VERTEX_NORMALS_WEIGHTING_TYPE_ANGLE ,
							 N_vertices );
	
	// 3. Setup return structure
	TriMeshVFN_ASP rtnStruct;
	rtnStruct.V = V;
	rtnStruct.F = F; 
	
	
	// 4. For each vertex
	size_t numV = V.rows();
	Eigen::Vector3d currNorm;
	Eigen::Vector3d currVrtx;
	for( size_t i = 0 ; i < numV ; i++ ){
		// 5. Calculate a vector that is 'shrinkDistance' in the OPPOSITE direction of the vertex normal  &&  6. Subtract and store
		currVrtx = V.row(i);
		currNorm = N_vertices.row(i);
		rtnStruct.V.row(i) = currVrtx - ( currNorm * shrinkDistance );
	}
	
	rtnStruct.N = N_from_VF( rtnStruct.V , F ); // It really doesn't matter that trinorms will have changed , but store for completeness
	rtnStruct.center = original.center;
	rtnStruct.axis   = original.axis;
	rtnStruct.type   = original.type;
	
	// 6. Return structure
	return rtnStruct;
}

TriMeshVFN_ASP expand_VFN_from_center( const TriMeshVFN_ASP& original , double factor ){
	size_t numV = original.V.rows();
	TriMeshVFN_ASP rtnMesh = copy_TriMeshVFN_ASP( original );
	Eigen::Vector3d currVrtx;
	Eigen::Vector3d currOfst;
	Eigen::Vector3d xpandPnt;
	// 1. For each of the vertices
	for( size_t i = 0 ; i < numV ; i++ ){
		// 2. Construct the vector from the center to the vertex
		currVrtx = original.V.row(i);
		currOfst = currVrtx - original.center;
		// 3. Construct a new point that is 'factor' times the distance from the center along the same line
		xpandPnt = original.center + currOfst * factor;
		// 4. Store the new point
		rtnMesh.V.row(i) = xpandPnt;
	}
	// 5. Return the expanded mesh
	return rtnMesh;
}

TriMeshVFN_ASP expand_VFN_from_axis( const TriMeshVFN_ASP& original , double factor ){
	// NOTE: This function assumes that 'original' has both the 'center' and 'axis' members populated
	size_t numV = original.V.rows();
	TriMeshVFN_ASP rtnMesh = copy_TriMeshVFN_ASP( original );
	Eigen::Vector3d currVrtx;
	Eigen::Vector3d currOfst;
	Eigen::Vector3d projcPnt;
	Eigen::Vector3d xpandPnt;
	// 1. For each of the vertices
	for( size_t i = 0 ; i < numV ; i++ ){
		// 2. Project the point onto the axis
		currVrtx = original.V.row(i);
		projcPnt = pnt_proj_onto_ray( currVrtx , original.center , original.axis );
		// 3. Construct the vector from the axis point to the vertex
		currOfst = currVrtx - projcPnt;
		// 4. Construct a new point that is 'factor' times the distance from the axis point along the same line
		xpandPnt = projcPnt + currOfst * factor;
		// 5. Store the new point
		rtnMesh.V.row(i) = xpandPnt;
	}
	// 5. Return the expanded mesh
	return rtnMesh;
}

TriMeshVFN_ASP shift_VFN_along_axis( const TriMeshVFN_ASP& original , double distance ){
	// NOTE: Both 'center' and 'axis' will be the same as the 'original'
	TriMeshVFN_ASP rtnMesh = copy_TriMeshVFN_ASP( original ); 
	Eigen::Vector3d shiftVec = original.axis.normalized() * distance;
	shift_V_inplace( rtnMesh.V , shiftVec );
	return rtnMesh;
}

// __ End Mesh Transform __

double furthest_extent_from_radius_in_dir( const TriMeshVFN_ASP& original , const Eigen::Vector3d& direction ){
	// Return the furthest from 'center'
	
	bool SHOWDEBUG = false; // if( SHOWDEBUG ){  cout << "" << endl;  }
	
	size_t len = original.V.rows();
	
	if( SHOWDEBUG ){  sep( "furthest_extent_from_radius_in_dir" , 3 );
					  cout << "Got a mesh with " << len << " rows" << endl;  }
	
	Eigen::Vector3d currPnt;
	double magMax  = -BILLION_D ,
		   currMag = 0.0        ;
	// 1. For each vertex
	for( size_t i = 0 ; i < len ; i++ ){
		// 2. Get the magnitude in 'direction'
		currPnt = original.V.row(i);
		if( SHOWDEBUG ){  cout << "\tCurrent Point: ________ " << currPnt << endl;  }
		currMag = vec3_project_mag( ( currPnt - original.center ) , direction ); 
		if( SHOWDEBUG ){  cout << "\tExtent along direction: " << currMag << endl;  }
		// 3. Store if it is the farthest
		magMax = max( magMax , currMag );
		if( SHOWDEBUG ){  cout << "\tNew Max: ______________ " << magMax << endl;  }
	}
	return magMax;
}

double V_radius_from_center( const Eigen::MatrixXd& V , const Eigen::Vector3d& center ){
	size_t len = V.rows();
	Eigen::Vector3d currPnt;
	double magMax  = -BILLION_D ,
		   currMag = 0.0        ;
	// 1. For each vertex
	for( size_t i = 0 ; i < len ; i++ ){
		// 2. Get the magnitude in 'direction'
		currPnt = V.row(i);
		currMag = ( currPnt - center ).norm();
		// 3. Store if it is the farthest
		magMax = max( magMax , currMag );
	}
	return magMax;
}

Eigen::MatrixXd get_triangle_i( const Eigen::MatrixXd& V , const Eigen::MatrixXi& F , size_t i ){ 
	// Get the 'V' of 'i'th triangle in 'F'
	Eigen::MatrixXd rtnTri = Eigen::MatrixXd::Zero( 3 , V.cols() ); // Should be applicable to both 3D and 2D triangles
	if( i < F.rows() ){ // If we asked for a valid triangle
		for( size_t j = 0 ; j < 3 ; j++ ){
			rtnTri.row(j) = V.row( F(i,j) );
		}
	} // Otherwise triangle out of bounds, Return zeros
	return rtnTri; // Return the CCW vertices
}

bool F_indices_OK( Eigen::MatrixXd& V , Eigen::MatrixXi& F ){ 
	// Check that F does not refer to nonexistent rows
	llin Flen = F.rows() , 
		 Vlen = V.rows() ;
	for( llin i = 0 ; i < Flen ; i++ ){
		for( llin j = 0 ; j < 3 ; j++ ){
			if( F(i,j) >= Vlen  ){  return false;  }
		}
	}
	return true;
}

double surface_area( const TriMeshVFN_ASP& mesh ){
	// Return the sum of the areas of all the triangles in the mesh
	Eigen::MatrixXd currTri;
	size_t numTri    = mesh.F.rows();
	double totalArea = 0.0;
	for( size_t i = 0 ; i < numTri ; i++ ){
		currTri = get_triangle_i( mesh.V , mesh.F , i );
		totalArea += tri_area( currTri );
	}
	return totalArea;
}

RayHits ray_intersect_VFN( const Eigen::Vector3d& rayOrg , const Eigen::Vector3d& rayDir , const TriMeshVFN_ASP& mesh ){
	// Return all the intersection points with the mesh , and classify them as either entry or exit points
	
	bool SHOWDEBUG = false;
	
	RayHits rtnStruct;  rtnStruct.anyHits = false;
	size_t numTris = mesh.F.rows();
	Eigen::MatrixXd tri = Eigen::MatrixXd::Zero( 4 , 2 );
	Eigen::Vector3d xBasis , yBasis , zBasis ,
				    v0 , v1 , v2 , 
				    interPnt , 
				    temp ;
	Eigen::Vector2d	intPnt2D ,
					v0flt , v1flt , v2flt ;
	
	//  1. For each triangle
	for( size_t f_i = 0 ; f_i < numTris ; f_i++ ){
		// Get plane point and normal from the mesh
		v0 = mesh.V.row( mesh.F( f_i , 0 ) );
		zBasis = mesh.N.row( f_i );  zBasis.normalize();
		//  2. Calc the intersection point
		interPnt = line_intersect_plane( rayOrg , rayDir , v0 , zBasis , false ); // ray parallel to facet doesn't intersect in this context
		//  3. If a valid intersection was returned
		if( !is_err( interPnt ) ){
			
			if( SHOWDEBUG ){  cout << "Plane intersection at " << interPnt << endl;  }
			
			//  4. Determine if the intersection happens in front of the ray
			if(  ( interPnt - rayOrg ).dot( rayDir ) >= 0.0  ){
				
				if( SHOWDEBUG ){  cout << "Intersection in FRONT of ray!" << endl;  }
				
				//  5. Get plane basis vectors from the triangle
				v1 = mesh.V.row( mesh.F( f_i , 1 ) );
				v2 = mesh.V.row( mesh.F( f_i , 2 ) );
				xBasis = ( v1 - v0 ).normalized();
				yBasis = zBasis.cross( xBasis ).normalized();
				//  6. Project the point onto the plane
				temp = point_basis_change( interPnt , v0 , xBasis , yBasis , zBasis );
				intPnt2D << temp(0) , temp(1);
				//  7. Project the triangle onto the plane
				// v0 will always be at ( 0 , 0 ) , per above
				// v1 will always be at ( ( v1 - v0 ) * x , 0 ) , per above
				tri( 1 , 0 ) = ( v1 - v0 ).dot( xBasis );
				// v2 will have ( x , y )
				temp = point_basis_change( v2 , v0 , xBasis , yBasis , zBasis );
				tri( 2 , 0 ) = temp( 0 );
				tri( 2 , 1 ) = temp( 1 );
				// v3 return to v0 for cycle = ( 0 , 0 ) , easy!
			
				if( SHOWDEBUG ){  
					cout << "Flat Point: " << intPnt2D << endl
						 << "Flat Tri" << endl
						 << tri << endl;
				}
				
				//  8. point in poly test , If the point is inside the triangle
				if(  point_in_poly_w( intPnt2D , tri )  ){
					
					if( SHOWDEBUG ){  cout << "Point INSIDE tri!"  << endl
										   << "Direction Dot: " << zBasis.dot( rayDir ) << endl;  }
					if( !rtnStruct.anyHits ){  rtnStruct.anyHits = true;  }
					//  9. Dot the ray with the tri norm, if negative
					if( zBasis.dot( rayDir ) < 0 ){
						// 10. else append to entrances
						rtnStruct.enter = copy_V_plus_row( rtnStruct.enter , interPnt );
						rtnStruct.n_Metric = copy_column_plus_dbbl( rtnStruct.n_Metric , angle_between( -rayDir , zBasis ) );
						if( SHOWDEBUG ){  
							cout << "n_Metric"  << endl
								 << rtnStruct.n_Metric << endl;
						}
					}else{ 
						// 11. Append to exits
						rtnStruct.exit  = copy_V_plus_row( rtnStruct.exit  , interPnt );
						rtnStruct.x_Metric = copy_column_plus_dbbl( rtnStruct.x_Metric , angle_between(  rayDir , zBasis ) );
						if( SHOWDEBUG ){  
							cout << "x_Metric"  << endl
								 << rtnStruct.x_Metric << endl;
						}
					}
				}else{ // else plane intersection outside of tri bounds, no action
					if( SHOWDEBUG ){  cout << "Point OUTSIDE tri!"  << endl;  }
				}
			}else{ // else triangle is behind ray, no collision
				if( SHOWDEBUG ){  cout << "Intersection BEHIND ray!" << endl;  }
			}
		} // else the ray missed the triangle plane
		if( SHOWDEBUG ){  cout << endl;  }
	}
	if( SHOWDEBUG ){  cout << endl;  }
	// 12. Return all hits
	return rtnStruct;
}

Eigen::Vector3d ray_intersect_AABB( const Eigen::Vector3d& origin , const Eigen::Vector3d& dir , const Eigen::MatrixXd& aabb ){
	/* 1. Fast Ray-Box Intersection
	      by Andrew Woo
	      from "Graphics Gems", Academic Press, 1990
	      URL: https://web.archive.org/web/20090803054252/http://tog.acm.org/resources/GraphicsGems/gems/RayBox.c
	   2. Adapted by Eric Haines
	      URL: https://github.com/erich666/GraphicsGems/blob/master/gems/RayBox.c
	   3. Adapted for C++ / Eigen by James Watson
	      URL: https://bitbucket.org/robot-learning/asm_seq_plan_3d/src/master/src/Motion_Cost/src/MathGeo_ASP.cpp
	*/
	// Return the last point at which the ray intersects the AABB , otherwise return ( NaN , NaN , NaN )
	
	size_t NUMDIM     = 3 , 
		   RIGHT      = 0 , 
		   LEFT /*-*/ = 1 , 
		   MIDDLE     = 2 , 
		   i /* -- */ = 0 ,
		   whichPlane = 0 ;
	bool inside = true;
	size_t quadrant[ NUMDIM ];
	double maxT[NUMDIM];
	double candidatePlane[NUMDIM];
	
	Eigen::Vector3d minB = aabb.row(0);
	Eigen::Vector3d maxB = aabb.row(1);
	
	Eigen::Vector3d coord;

	/* Find candidate planes; this loop can be avoided if rays cast all from the eye ( assume perpsective view ) */
	for( i = 0 ; i < NUMDIM ; i++ ){
		if( origin(i) < minB(i) ){
			quadrant[i] = LEFT;
			candidatePlane[i] = minB(i);
			inside = false;
		}else if( origin(i) > maxB(i) ){
			quadrant[i] = RIGHT;
			candidatePlane[i] = maxB(i);
			inside = false;
		}else{
			quadrant[i] = MIDDLE;
		}
	}

	/* Ray origin inside bounding box */
	if( inside ){
		//~ coord = origin;
		//~ return (TRUE);
		return origin;
	}

	/* Calculate T distances to candidate planes */
	for( i = 0 ; i < NUMDIM ; i++ ){
		if( quadrant[i] != MIDDLE && dir[i] != 0.0 ) 
			maxT[i] = ( candidatePlane[i] - origin(i) ) / dir(i);
		else
			maxT[i] = -1.0;
	}

	/* Get largest of the maxT's for final choice of intersection */
	whichPlane = 0;
	for( i = 1 ; i < NUMDIM ; i++ ){
		if( maxT[ whichPlane ] < maxT[i] ){  whichPlane = i;  }
	}

	/* Check final candidate actually inside box */
	if( maxT[ whichPlane ] < 0.0){  return err_vec3();  }
	
	for( i = 0 ; i < NUMDIM ; i++ ){
		if( whichPlane != i ){
			coord(i) = origin(i) + maxT[ whichPlane ] * dir(i);
			if(  ( coord(i) < minB(i) )  ||  ( coord(i) > maxB(i) )  ){  return err_vec3();  }
		} else {
			coord(i) = candidatePlane[i];
		}
	}
	return coord;				/* ray hits box */
} 

RayHits ray_intersect_TargetVFN( const Eigen::Vector3d& rayOrg , const Eigen::Vector3d& rayDir , const TargetVFN_ASP& target ){
	// Fast collision recording between ray and mesh-target
	// NOTE: This function assumes that 'target' AABB and mesh concur and are up to date
	RayHits rtnStruct;  rtnStruct.anyHits = false;
	
	// 1. If there is a collision with the bounding box, then we may proceed with the more costly computation of mesh collisions
	if(  !is_err(  ray_intersect_AABB( rayOrg , rayDir , target.aabb )  )  ){
		rtnStruct = ray_intersect_VFN( rayOrg , rayDir , target.mesh );
	}
	
	return rtnStruct;
}

RayHits ray_intersect_CollsnVFN( const Eigen::Vector3d& rayOrg , const Eigen::Vector3d& rayDir , const CollisionVFN_ASP& target ){
	// Fast collision recording between ray and mesh-target
	// NOTE: This function assumes that 'target' AABB and mesh concur and are up to date
	RayHits rtnStruct;  rtnStruct.anyHits = false;
	// 1. If there is a collision with the bounding box, then we may proceed with the more costly computation of mesh collisions
	if(  !is_err(  ray_intersect_AABB( rayOrg , rayDir , target.aabb )  )  ){
		rtnStruct = ray_intersect_VFN( rayOrg , rayDir , target.mesh );
	}
	return rtnStruct;
}

RayHits perforate_meshes_and_obtain_FILO_pairs( std::vector<TargetVFN_ASP>& meshTargets , 
												Eigen::MatrixXd rayOrgs , Eigen::MatrixXd rayDirs ){
	// Shoot all 'meshTargets' with all specified rays (PEW PEW PEW) and 
	// return a list of { First In (closest to rayOrg) , Last Out (furthest from rayOrg) } contact points
	// NOTE: This function returns a special case of 'RayHits' in which corrosponding rows of { 'entry' , 'exit' } form a FILO pair
	// NOTE: This function assumes that 'rayOrgs' and 'rayDirs' are of identical shape ( N , 3 ) for N rays
	
	bool SHOWDEBUG = false;
	
	RayHits rtnStruct;  rtnStruct.anyHits = false;
	size_t numRays = rayOrgs.rows()     , 
		   numTrgt = meshTargets.size() ;
	Eigen::Vector3d rayOrg;
	Eigen::Vector3d rayDir;
	Eigen::Vector3d nearest;
	Eigen::Vector3d farthst;
	RayHits currHits;
	bool foundHits = false;
	double angleFI , 
		   angleLO ;
	IndexDbblResult clsResult ,
					farResult ;
	
	if( SHOWDEBUG ){  cout << "About to iterate ..." << endl;  }
	
	//  1. For each ray
	for( size_t ray_i = 0 ; ray_i < numRays ; ray_i++ ){
		Eigen::MatrixXd entrs;  Eigen::MatrixXd n_Mtrc;
		Eigen::MatrixXd exits;  Eigen::MatrixXd x_Mtrc;
		rayOrg = rayOrgs.row( ray_i );
		rayDir = rayDirs.row( ray_i );
		
		if( SHOWDEBUG ){  cout << "Ray , Origin: " << rayOrg << " , Direction: " << rayDir << endl;  }
		
		foundHits = false;
		//  2. For each target
		for( size_t trg_j = 0 ; trg_j < numTrgt ; trg_j++ ){
			if( SHOWDEBUG ){  cout << "Target " << trg_j + 1 << " of " << numTrgt << ": ";  }
			//  3. Evaluate ray collision
			currHits = ray_intersect_TargetVFN( rayOrg , rayDir , meshTargets[ trg_j ] );
			if( currHits.anyHits ){
				if( SHOWDEBUG ){  cout << "Hits found , Accumulating points ..." << endl;  }
				if( !foundHits ){  
					if( SHOWDEBUG ){  cout << "Hit flag set." << endl;  }
					foundHits = true;  
				}
				
				//  4. Accumulate entries
				if( SHOWDEBUG ){  cout << "Accumulate " << currHits.enter.rows() << " entries ..." << endl;  }
				entrs  = vstack( entrs  , currHits.enter    );
				n_Mtrc = vstack( n_Mtrc , currHits.n_Metric );
				
				//  5. Accumulate exits
				if( SHOWDEBUG ){  cout << "Accumulate " << currHits.exit.rows() << " exits ..." << endl;  }
				exits  = vstack( exits  , currHits.exit     );
				x_Mtrc = vstack( x_Mtrc , currHits.x_Metric );
				
			}else{  if( SHOWDEBUG ){  cout << "No hits found" << endl;  }  }
		}
		if(  foundHits  &&  entrs.rows() > 0  &&  exits.rows() > 0  ){
			if( SHOWDEBUG ){  cout << "Fetch FILO points ..." << endl;  }
			
			//  6. Determine nearest entry
			if( SHOWDEBUG ){  cout << "Find FI point ..." << endl;  }
			
			clsResult = closest_point_to_sq(   entrs , rayOrg );
			if( SHOWDEBUG ){  cout << "Fetch nearest ..." << endl;  }
			nearest   = entrs.row( clsResult.index );
			if( SHOWDEBUG ){  cout << "Store angle ..." << endl;  }
			angleFI   = n_Mtrc( clsResult.index , 0 );
			if( SHOWDEBUG ){  cout << "FI: " << nearest << endl;  }
			
			//  7. Determine furthest exit
			if( SHOWDEBUG ){  
				cout << "Find LO point ..." << endl
					 << "There are " << exits.rows() << " exits." << endl;
			}
			farResult = farthest_point_from_sq( exits , rayOrg );
			farthst   = exits.row( farResult.index );
			angleLO   = x_Mtrc( farResult.index , 0 );
			if( SHOWDEBUG ){  cout << "LO: " << farthst << endl;  }
			
			//  8. Mark collision true
			if( !rtnStruct.anyHits ){  
				if( SHOWDEBUG ){  cout << "Set flag ..." << endl;  }
				rtnStruct.anyHits = true;  
			}
			
			//  9. Add pair to result, if it exists
			if( SHOWDEBUG ){  cout << "Add FILO pair ..." << endl;  }
			rtnStruct.enter    = copy_V_plus_row( rtnStruct.enter , nearest );
			rtnStruct.n_Metric = copy_column_plus_dbbl( rtnStruct.n_Metric , angleFI );
			rtnStruct.exit     = copy_V_plus_row( rtnStruct.exit  , farthst );
			rtnStruct.x_Metric = copy_column_plus_dbbl( rtnStruct.x_Metric , angleLO );
		}
		if( SHOWDEBUG ){  cout << endl;  }
	}
	if( SHOWDEBUG ){  cout << "Return" << endl;  }
	// 10. Return result
	return rtnStruct;
}

Eigen::MatrixXd extract_V_from_TargetVFN( std::vector<TargetVFN_ASP>& partTargets ){
	Eigen::MatrixXd rtnV;
	size_t len = partTargets.size();
	for( size_t i = 0 ; i < len ; i++ ){  rtnV = vstack( rtnV , partTargets[i].mesh.V );  }
	return rtnV;
}

TriMeshVFN_ASP merge_meshes( const TriMeshVFN_ASP& op1 , const TriMeshVFN_ASP& op2 ){
	// Naively combine traingles from two different meshes
	
	bool SHOWDEBUG = false; // if( SHOWDEBUG ){  cout << "" << endl;  }
	
	if( SHOWDEBUG ){  cout << "Copy meshes ..." << endl;  }
	TriMeshVFN_ASP rtnMesh = copy_TriMeshVFN_ASP( op1 );
	TriMeshVFN_ASP tmpMesh = copy_TriMeshVFN_ASP( op2 );
	int op1numVerts  = op1.V.rows() , // Offset for the added mesh
		op2numFacets = op2.F.rows() ,
	    numCols      = 3            ; 
	    
	if( SHOWDEBUG ){  cout << "Offset facets ..." << endl;  }
	// Offset facets
	for( int i = 0 ; i < op2numFacets ; i++ ){
		for( int j = 0 ; j < numCols ; j ++ ){
			tmpMesh.F( i , j ) += op1numVerts; // Offset the facets
		}
	}
	if( SHOWDEBUG ){  cout << "Copy V ..." << endl;  }
	rtnMesh.V      = vstack( rtnMesh.V , op2.V     ); 
	if( SHOWDEBUG ){  cout << "Copy F ..." << endl;  }
	rtnMesh.F	   = vstack( rtnMesh.F , tmpMesh.F ); 
	if( SHOWDEBUG ){  cout << "Copy N ..." << endl;  }
	rtnMesh.N      = vstack( rtnMesh.N , op2.N     );
	rtnMesh.center = err_vec3(); 
	rtnMesh.axis   = err_vec3(); 
	rtnMesh.type   = GENERIC;
	
	return rtnMesh;
}

// == Centroid Operations ==

VolumeCentroid get_VolumeCentroid_from_V_F( Eigen::MatrixXd& V , Eigen::MatrixXi& F ){
	size_t i       = 0        ,
		   numTris = F.rows() ,
		   j       = 0        ;
	std::vector<VolumeCentroid> tetPointVolumes;
	VolumeCentroid tempVol;
	int centerSign;
	// 1. Define a central point
	Eigen::Vector3d centralPnt = get_average_V( V );
	Eigen::Vector3d triNorm;  Eigen::Vector3d v0;  Eigen::Vector3d v1;  Eigen::Vector3d v2;
	// 2. For every triangle
	for( i = 0 ; i < numTris ; i++ ){
		// 3. Form a tet from every tri
		v0 = V.row( F( i , 0 ) );
		v1 = V.row( F( i , 1 ) );
		v2 = V.row( F( i , 2 ) );
		Tetrahedron_ASP temp{ v0 , v1 , v2 , centralPnt };
		// 4. Get the triangle norm
		triNorm = get_CCW_tri_norm( v0 , v1 , v2 );
		// 5. Determine what side the central point is with respect to the triangle
		centerSign = sign( dist_to_plane( triNorm , v0 , centralPnt ) );
		// 6. Accumulate a positive or negative volume and centroid
		tempVol = get_tetrahedron_VolumeCentroid( temp );
		// tempVol.volume *= (double) centerSign; // This might turn out to be a double negative!
		tetPointVolumes.push_back( tempVol );
	}
	return get_VolumeCentroid_from_discrete_volume_centroids( tetPointVolumes );
}

VolumeCentroid get_VolumeCentroid_from_discrete_volume_centroids( std::vector<VolumeCentroid>& discretePointVolumes ){
	size_t i      = 0                           ,
		   numPts = discretePointVolumes.size() ; 
	VolumeCentroid rtnStruct;  rtnStruct.volume = 0.0d;  Eigen::Vector3d temp;  temp << 0 , 0 , 0;  rtnStruct.centroid = temp;
	// 1. Pass 1 : Get the total volume
	for( i = 0 ; i < numPts ; i++ ){ rtnStruct.volume += discretePointVolumes[i].volume; }
	// 2. Pass 2 : Get the weighted average of centroids
	for( i = 0 ; i < numPts ; i++ ){ 
		rtnStruct.centroid += discretePointVolumes[i].centroid * ( discretePointVolumes[i].volume / rtnStruct.volume );
	}
	return rtnStruct;
}

double get_tetrahedron_volume( Eigen::Vector3d& v0 , 
							   Eigen::Vector3d& v1 ,
							   Eigen::Vector3d& v2 ,
							   Eigen::Vector3d& v3 ){
	// URL , Volume of a tetrahedron: http://mathworld.wolfram.com/Tetrahedron.html
	Eigen::MatrixXd rep = Eigen::MatrixXd::Zero( 4 , 4 );
	rep << v0(0) , v0(1) , v0(2) , 1.0d , 
		   v1(0) , v1(1) , v1(2) , 1.0d , 
		   v2(0) , v2(1) , v2(2) , 1.0d , 
		   v3(0) , v3(1) , v3(2) , 1.0d ;
	return rep.determinant() * 1.0d / 6.0d; // TODO: Find out if this will return negative volumes
}

double get_tetrahedron_volume( Tetrahedron_ASP tet ){ return get_tetrahedron_volume( tet.v0 , tet.v1 , tet.v2 , tet.v3 ); }

Eigen::Vector3d get_tetrahedron_centroid( Eigen::Vector3d& v0 , 
										  Eigen::Vector3d& v1 ,
										  Eigen::Vector3d& v2 ,
										  Eigen::Vector3d& v3 ){ return ( v0 + v1 + v2 + v3 ) / 4.0; } // It's just the average

Eigen::Vector3d get_tetrahedron_centroid( Tetrahedron_ASP tet ){ return get_tetrahedron_centroid( tet.v0 , tet.v1 , tet.v2 , tet.v3 ); }

VolumeCentroid get_tetrahedron_VolumeCentroid( Eigen::Vector3d& v0 , 
											   Eigen::Vector3d& v1 ,
											   Eigen::Vector3d& v2 ,
											   Eigen::Vector3d& v3 ){
	VolumeCentroid rtnStruct;  
	rtnStruct.volume   = get_tetrahedron_volume(   v0 , v1 , v2 , v3 );
	rtnStruct.centroid = get_tetrahedron_centroid( v0 , v1 , v2 , v3 );
	return rtnStruct;
}

VolumeCentroid get_tetrahedron_VolumeCentroid( Tetrahedron_ASP tet ){ 
	return get_tetrahedron_VolumeCentroid( tet.v0 , tet.v1 , tet.v2 , tet.v3 ); 
}

// __ End Centroid __


// == Convex Hull ==

TriMeshVFN_ASP V_to_ConvexHull_VFN( const Eigen::MatrixXd& Vertices ){
	size_t i       = 0               ,
		   j       = 0               ,
		   numRows = Vertices.rows() ,
		   numV    = 0               ,
		   numF    = 0               ;
	std::vector<quickhull::Vector3<double>> pointCloud; // Collection of points to form a hull around
	TriMeshVFN_ASP rtnStruct;
	// 1. Load the vertices into a quickhull array
	for( i = 0 ; i < numRows ; i++ ){
		pointCloud.push_back( quickhull::Vector3<double>( Vertices( i , 0 ) , Vertices( i , 1 ) , Vertices( i , 2 ) ) );
	}
	// 2. Create a hull from the array of points
	quickhull::QuickHull<double> qh;
	quickhull::ConvexHull<double> hull = qh.getConvexHull( pointCloud , true , false );
	// 3. Get the tri-mesh for the convex hull , Unpack the vertices and indices
	std::vector<size_t> indexBuffer                  = hull.getIndexBuffer(); 
	quickhull::VertexDataSource<double> vertexBuffer = hull.getVertexBuffer();
	// 4. Load V and F 
	numV = vertexBuffer.size();  numF = indexBuffer.size() / 3;
	Eigen::MatrixXd rtnV = Eigen::MatrixXd::Zero( numV , 3 );
	Eigen::MatrixXi rtnF = Eigen::MatrixXi::Zero( numF , 3 ); 
	Eigen::MatrixXd rtnN = Eigen::MatrixXd::Zero( numF , 3 );
	// Load the vertices
	for( i = 0 ; i < numV ; i++ ){
		rtnV.row(i) = Eig_vec3d_round_zero( hullV3d_to_eigV3d( vertexBuffer[ i ] ) );
	}
	// Load the facets
	for( i = 0 ; i < numF ; i++ ){
		for( j = 0 ; j < 3 ; j++ ){ rtnF( i , j ) = indexBuffer[ 3 * i + j ]; }
		// 5. Calc N 
		Eigen::Vector3d v0 = rtnV.row( rtnF( i , 0 ) );
		Eigen::Vector3d v1 = rtnV.row( rtnF( i , 1 ) );
		Eigen::Vector3d v2 = rtnV.row( rtnF( i , 2 ) );
		rtnN.row(i) = get_CCW_tri_norm( v0 , v1 , v2 );
	}
	// 6. Load matrices into the return structure , and return
	rtnStruct.V    = rtnV;
	rtnStruct.F    = rtnF;
	rtnStruct.N    = rtnN;
	rtnStruct.type = GENERIC;
	
	// 7. If the hull comes out with flipped tris , it will cause trouble with putdown poses	
	repair_convex_VFN( rtnStruct );
	
	if( !check_VFN_convexity( rtnStruct ) ){  sep( "WARN: CONVEX REPAIR FAILED!" , 16 , '!' );  }
	
	return rtnStruct;
}

// Convert an Eigen R3 vector to a quickhull R3 vector
quickhull::Vector3<double> eigV3d_to_hullV3d( Eigen::Vector3d& eigVec ){
	return quickhull::Vector3<double>( eigVec(0) , eigVec(1) , eigVec(2) );
}

// Convert an quickhull R3 vector to a Eigen R3 vector
Eigen::Vector3d hullV3d_to_eigV3d( quickhull::Vector3<double>& hullVec ){
	return Eigen::Vector3d( hullVec.x , hullVec.y , hullVec.z );
}

// 'const' version of the above
Eigen::Vector3d hullV3d_to_eigV3d( const quickhull::Vector3<double>& hullVec ){
	return Eigen::Vector3d( hullVec.x , hullVec.y , hullVec.z );
}

bool check_VFN_convexity( const TriMeshVFN_ASP& mesh ){
	// Return true if the trimesh is convex, otherwise return false
	Eigen::Vector3d pCentroid = vec_avg( mesh.V );
	Eigen::Vector3d triCen;
	Eigen::Vector3d outRay;
	Eigen::Vector3d triNorm;
	size_t meshLen = mesh.F.rows();
	double dotResult = 0.0;
	Eigen::MatrixXd tri = Eigen::MatrixXd::Zero( 3 , 3 );
	for( size_t i = 0 ; i < meshLen ; i++ ){
		for( size_t j = 0 ; j < 3 ; j++ ){ tri.row( j ) = mesh.V.row( mesh.F( i , j ) ); }
		triCen = vec_avg( tri );
		outRay = triCen - pCentroid;
		triNorm = mesh.N.row(i);
		dotResult = outRay.dot( triNorm );
		if( dotResult < 0.0 ){  return false;  }
	}
	return true;
}

void repair_convex_VFN( TriMeshVFN_ASP& mesh ){
	// Return true if the trimesh is convex, otherwise return false
	Eigen::Vector3d pCentroid = vec_avg( mesh.V );
	Eigen::Vector3d triCen;
	Eigen::Vector3d outRay;
	Eigen::Vector3d triNorm;
	size_t meshLen = mesh.F.rows();
	double dotResult = 0.0;
	size_t swap = 0;
	Eigen::MatrixXd tri = Eigen::MatrixXd::Zero( 3 , 3 );
	for( size_t i = 0 ; i < meshLen ; i++ ){
		for( size_t j = 0 ; j < 3 ; j++ ){ tri.row( j ) = mesh.V.row( mesh.F( i , j ) ); }
		triCen = vec_avg( tri );
		outRay = triCen - pCentroid;
		triNorm = mesh.N.row(i);
		dotResult = outRay.dot( triNorm );
		if( dotResult < 0.0 ){  
			// Swap two of the triangle vertices
			swap = mesh.F( i , 2 );
			mesh.F( i , 2 ) = mesh.F( i , 1 );
			mesh.F( i , 1 ) = swap;
			// Flip the normal
			mesh.N.row(i) = triNorm * -1.0;
		}
	}
}

// __ End Hull __


// == class Icosahedron_d ==

// Geometry based on Paul Bourke's excellent article:
//   Platonic Solids (Regular polytopes in 3D)
//   http://astronomy.swin.edu.au/~pbourke/polyhedra/platonic/

// ~ Constructors & Destructors ~
void Icosahedron_d::_init( double rad , const Eigen::Vector3d& cntr ){
	center = cntr;
	radius = rad;
	a = ( radius / ratio ) * 0.5;
	b = ( radius / ratio ) / ( 2.0f * phi );
	V = Eigen::MatrixXd::Zero( 12 ,  3 ); // Points of the mesh
	F = Eigen::MatrixXi::Zero( 20 ,  3 ); // Facets corresponding to the points V
	
	// Define the icosahedron's 12 vertices:
	V.row(  0 ) = cntr + Eigen::Vector3d(  0 ,  b , -a );
	V.row(  1 ) = cntr + Eigen::Vector3d(  b ,  a ,  0 );
	V.row(  2 ) = cntr + Eigen::Vector3d( -b ,  a ,  0 );
	V.row(  3 ) = cntr + Eigen::Vector3d(  0 ,  b ,  a );
	V.row(  4 ) = cntr + Eigen::Vector3d(  0 , -b ,  a );
	V.row(  5 ) = cntr + Eigen::Vector3d( -a ,  0 ,  b );
	V.row(  6 ) = cntr + Eigen::Vector3d(  0 , -b , -a );
	V.row(  7 ) = cntr + Eigen::Vector3d(  a ,  0 , -b );
	V.row(  8 ) = cntr + Eigen::Vector3d(  a ,  0 ,  b );
	V.row(  9 ) = cntr + Eigen::Vector3d( -a ,  0 , -b );
	V.row( 10 ) = cntr + Eigen::Vector3d(  b , -a ,  0 );
	V.row( 11 ) = cntr + Eigen::Vector3d( -b , -a ,  0 );
	
	// Define the icosahedron's 20 triangular faces:
	//   CCW            ||  CW
    F <<  2 ,  1 ,  0 , //~  0 ,  1 ,  2 ,
          1 ,  2 ,  3 , //~  3 ,  2 ,  1 ,
          5 ,  4 ,  3 , //~  3 ,  4 ,  5 ,
          4 ,  8 ,  3 , //~  3 ,  8 ,  4 ,
          7 ,  6 ,  0 , //~  0 ,  6 ,  7 ,
          6 ,  9 ,  0 , //~  0 ,  9 ,  6 ,
         11 , 10 ,  4 , //~  4 , 10 , 11 ,
         10 , 11 ,  6 , //~  6 , 11 , 10 ,
          9 ,  5 ,  2 , //~  2 ,  5 ,  9 ,
          5 ,  9 , 11 , //~ 11 ,  9 ,  5 ,
          8 ,  7 ,  1 , //~  1 ,  7 ,  8 ,
          7 ,  8 , 10 , //~ 10 ,  8 ,  7 ,
          2 ,  5 ,  3 , //~  3 ,  5 ,  2 ,
          8 ,  1 ,  3 , //~  3 ,  1 ,  8 ,
          9 ,  2 ,  0 , //~  0 ,  2 ,  9 ,
          1 ,  7 ,  0 , //~  0 ,  7 ,  1 ,
         11 ,  9 ,  6 , //~  6 ,  9 , 11 ,
          7 , 10 ,  6 , //~  6 , 10 ,  7 ,
          5 , 11 ,  4 , //~  4 , 11 ,  5 ,
         10 ,  8 ,  4 ; //~  4 ,  8 , 10 ;
}

Icosahedron_d::Icosahedron_d(){ _init( 1.0d , Eigen::Vector3d( 0.0d , 0.0d , 0.0d ) ); } // Default constructor

Icosahedron_d::Icosahedron_d( double rad , const Eigen::Vector3d& cntr ){ _init( rad , cntr ); } // Parameter constructor

Icosahedron_d::~Icosahedron_d(){ /* Nothing to do here! */ } // Destructor

// ~ Getters ~
Eigen::MatrixXd& Icosahedron_d::get_vertices(){ return V; }
Eigen::MatrixXi& Icosahedron_d::get_facets(){   return F; };

// __ End Icosahedron_d __


// == class Sphere_d ==

// = Sphere Helpers =

bool test_against_constraint_norms( const Eigen::MatrixXd& constraintNorms , const Eigen::Vector3d& rayDir ){
	// Return true if ray does not violate constraints
	
	bool SHOWDEBUG = false; // if( SHOWDEBUG ){  cout << "" << endl;  }
	
	if( SHOWDEBUG ){  sep( "test_against_constraint_norms" );  }
	
	size_t numNorms = constraintNorms.rows();
	Eigen::Vector3d currDir;
	double projMag = 0.0;
	
	for( size_t i = 0 ; i < numNorms ; i++ ){
		currDir = constraintNorms.row(i);
		projMag = round_zero( vec3_project_mag( rayDir , currDir ) );
		if( SHOWDEBUG ){  cout << "Projected Magnitude: " << projMag << endl;  }
		
		if( projMag <  0.0  ){  
			if( SHOWDEBUG ){  cout << "FAIL" << endl;  }
			return false;  
		}
	}
	if( SHOWDEBUG ){  cout << "PASS" << endl;  }
	return true;
}

bool test_against_constraint_planes( const Eigen::MatrixXd& constraintNorms , const Eigen::MatrixXd& constraintPnts , const Eigen::Vector3d& queryPnt ){
	// Return true if 'queryPnt' can be found on the positive side of all planes defined by 'constraintNorms' and 'constraintPnts'
	// NOTE: This function assumes that both 'constraintNorms' and 'constraintPnts' are both Nx3 where N is the number of constraint planes
	size_t numNorms = constraintNorms.rows();
	Eigen::Vector3d currPnt;
	Eigen::Vector3d planPnt;
	Eigen::Vector3d currNrm;
	for( size_t i = 0 ; i < numNorms ; i++ ){
		currNrm = constraintNorms.row(i);
		planPnt = constraintPnts.row(i);
		currPnt = queryPnt - planPnt;
		if(  round_zero( vec3_project_mag( currPnt , currNrm ) )  <  0.0  ){  return false;  }
	}
	return true;
}

RayHits concat_hits( const RayHits& op1 , const RayHits& op2 ){
	// Collect hits info from two response objects and collect them into one
	// NOTE: This function assumes that concatenating each of the matrices in the response objects individually does not create a row 
	//       misalignment that causes problems for the client code
	// NOTE: This function assumes that a hit in either of the operands signifies a hit in the concatenated structure
	
	bool SHOWDEBUG = false; // if( SHOWDEBUG ){  cout << "" << endl;  }
	if( SHOWDEBUG ){  cout << "\tConcat op1 and op2 ..." << endl 
						   << "\top1 has " << op1.enter.rows() << " pairs" << endl
						   << "\top2 has " << op2.enter.rows() << " pairs" << endl;  }
	
	RayHits rtnStruct;
	rtnStruct.anyHits  =       ( op1.anyHits || op2.anyHits  ); // Were there any intersections recorded in either of the structures??
	rtnStruct.enter    = vstack( op1.enter    , op2.enter    ); // Row-list of entry points
	rtnStruct.exit     = vstack( op1.exit     , op2.exit     ); // Row-list of exit points
	rtnStruct.n_Metric = vstack( op1.n_Metric , op2.n_Metric ); // Generic entry metrics to be populated by client code (e.g. grasp pair angles)
	rtnStruct.x_Metric = vstack( op1.x_Metric , op2.x_Metric ); // Generic exit  metrics to be populated by client code (e.g. grasp pair angles)
	return rtnStruct;
}

std::vector<Pose_ASP> translate_pose_by_sequence( const Pose_ASP& original , const Eigen::MatrixXd& seqPos ){
	// NOTE: This function assumes that 'seqPos' is an N x 3 matrix
	std::vector<Pose_ASP> rtnVec;
	size_t len = seqPos.rows();
	Eigen::Vector3d currOffset;
	for( size_t i = 0 ; i < len ; i++ ){
		currOffset = seqPos.row(i);
		rtnVec.push_back(  Pose_ASP{ original.position + currOffset , original.orientation }  );
	}
	return rtnVec;
}

SuccessPoints empty_SuccessPoints(){
	SuccessPoints rtnStruct;
	rtnStruct.success    = false;
	rtnStruct.travelDist = 0.0;
	rtnStruct.bestDir    = err_vec3();
	//~ rtnStruct.goodSeqPos = err_matx();
	return rtnStruct;
}

// _ End Helpers _

size_t FACETSTODIVIDE = 10; // DEBUG

// ~ Init ~
void Sphere_d::_init( Icosahedron_d icos , size_t subdivision ){
	
	bool SHOWDEBUG = false; // if( SHOWDEBUG ){  cout << "" << endl;  }
	
	if( SHOWDEBUG ){  cout << "Init vars ..." << endl;  }
	size_t i     = 0 , 
	       j     = 0 ,
	       row   = 0 ,
	       x     = 0 ,
	       y     = 0 ,
	       count = 0 ;
	Eigen::Vector3d v0 , v1 , v2 , xTri , yTri , temp;
	if( SHOWDEBUG ){  cout << "Allocate ..." << endl;  }
	// 1. The number of facets is       20 * ( tri( sub ) + tri( sub - 1 ) )
	size_t numF = 20 * ( tri_num( subdivision ) + tri_num( subdivision - 1 ) );
	// 2. The number of vertices is 3 * 20 * ( tri( sub ) + tri( sub - 1 ) )
	size_t numV = 3 * numF;
	V = Eigen::MatrixXd::Zero( numV , 3 );
	F = Eigen::MatrixXi::Zero( numF , 3 );
	// 3. Fetch V and F
	Eigen::MatrixXd V_icos = icos.get_vertices();
	Eigen::MatrixXi F_icos = icos.get_facets();
	// 4. For each facet
	for( i = 0 ; i < 20 ; i++ ){
		if( SHOWDEBUG ){  cout << "Subdividing facet " << i+1 << " of 20" << endl;  }
	//~ for( i = 0 ; i < FACETSTODIVIDE ; i++ ){
		// 5. Construct basis vectors of length ( edge length ) / sub
		v0 = V_icos.row( F_icos( i , 0 ) );  v1 = V_icos.row( F_icos( i , 1 ) );  v2 = V_icos.row( F_icos( i , 2 ) );
		xTri = ( v1 - v0 ) / (double) subdivision;  yTri = ( v2 - v0 ) / (double) subdivision;	
		
		//~ cout << "DEBUG , xTri" << endl << xTri << endl;
		//~ cout << "DEBUG , yTri" << endl << yTri << endl;
			
		// 6. Construct subdivision vertices ( These must be added in CCW order )
		for( row = 1 ; row <= subdivision ; row++ ){
			//~ for( j = row - 1 ; j > 0 ; j-- ){ // Construct the v0-pointing tris
			for( j = row ; j > 0 ; j-- ){ // Construct the v0-pointing tris
				V.row( count ) = v0 + vec3d_from_arbitrary_2D_basis( (double) j     , (double) row - j     , xTri , yTri ); count++;
				V.row( count ) = v0 + vec3d_from_arbitrary_2D_basis( (double) j - 1 , (double) row - j + 1 , xTri , yTri ); count++;
				V.row( count ) = v0 + vec3d_from_arbitrary_2D_basis( (double) j - 1 , (double) row - j     , xTri , yTri ); count++;
			}
			for( j = row - 1 ; j > 0 ; j-- ){ // Construct the anti-v0-pointing tris
			//~ for( j = row - 1 ; j > 0 ; j-- ){ // Construct the anti-v0-pointing tris
				V.row( count ) = v0 + vec3d_from_arbitrary_2D_basis( (double) j     , (double) row - 1 - j     , xTri , yTri ); count++;
				V.row( count ) = v0 + vec3d_from_arbitrary_2D_basis( (double) j     , (double) row - 1 - j + 1 , xTri , yTri ); count++;
				V.row( count ) = v0 + vec3d_from_arbitrary_2D_basis( (double) j - 1 , (double) row - 1 - j + 1 , xTri , yTri ); count++;
			}
		}
	}
	
	//~ cout << "DEBUG: (double) subdivision: " << (double) subdivision << endl;
	
	if( SHOWDEBUG ){  cout << "Enumerating facets ..." << endl;  }
	// 7. Facets are just sequential V indices in groups of 3s
	for( i = 0 ; i < numF ; i++ ){
		for( j = 0 ; j < 3 ; j++ ){
			F( i , j ) = i * 3 + j;
		}
	}
	if( SHOWDEBUG ){  cout << "Transforming vertices ..." << endl;  }
	// 8. Transform vertices , facets are unchanged
	_transform_V();
}

void Sphere_d::_transform_V(){ // Pull all the vertices out to the surface of the sphere
	bool SHOWDEBUG = false; // if( SHOWDEBUG ){  cout << "" << endl;  }
	size_t i    = 0        ,
		   numV = V.rows() ;
	Eigen::Vector3d pnt;
	// 0. For every vertex
	for( i = 0 ; i < numV ; i++ ){
		if( SHOWDEBUG ){  cout << "Vertex " << i+1 << " of " << numV << endl;  }
	//~ for( i = 0 ; i < FACETSTODIVIDE * 3 * ( tri_num( sub ) + tri_num( sub - 1 ) ) ; i++ ){
		// 1. Construct a unit vector that points from the center to the vertex
		// 2. Extend the vector to the surface of the sphere
		pnt = V.row(i);
		if( SHOWDEBUG ){  cout << "Original Vertex: _ " << pnt << endl;  }
		V.row(i) = ( pnt - baseIcos.center ).normalized() * radius + baseIcos.center;
		//         ^-- direction                   ^-- magnitude
		if( SHOWDEBUG ){  cout << "Transformed Vertex:" << V.row(i) << endl;  }
	}
}

// ~ Constructors & Destructors ~
Sphere_d::Sphere_d(){ // ---------------------------------- Default constructor
	baseIcos = Icosahedron_d();
	radius   = baseIcos.radius;
	sub      = 5;
	_init( baseIcos , sub );
}

Sphere_d::Sphere_d( double rad , const Eigen::Vector3d& cntr , size_t subdivision ){ // Parameter constructor
	bool SHOWDEBUG = false; // if( SHOWDEBUG ){  cout << "" << endl;  }
	radius   = rad;
	if( SHOWDEBUG ){  cout << "Building ICOS ..." << endl;  }
	baseIcos = Icosahedron_d( rad , cntr );
	sub      = subdivision;
	center   = cntr;
	if( SHOWDEBUG ){  cout << "Expanding icos to sphere ..." << endl;  }
	_init( baseIcos , sub );
}

Sphere_d::Sphere_d( Icosahedron_d icos , size_t subdivision ){
	baseIcos = icos;
	radius   = baseIcos.radius;
	sub      = subdivision;
	center   = baseIcos.center;
	_init( baseIcos , sub );
}

Sphere_d::~Sphere_d(){ // --------------------------------- Destructor
	// nothing to do here!
}

// ~ Getters ~
Eigen::MatrixXd& Sphere_d::get_vertices(){ return V; }
Eigen::MatrixXi& Sphere_d::get_facets(){   return F; }

TriMeshVFN_ASP Sphere_d::get_mesh(){
	// Return a trimesh representing the sphere
	TriMeshVFN_ASP rtnStruct;
	rtnStruct.V      = V;
	rtnStruct.F      = F;
	rtnStruct.N      = N_from_VF( V , F );
	rtnStruct.center = center;
	rtnStruct.type   = SPHERICAL;
	return rtnStruct;
}

// ~ Surface slicing ~
TriMeshVFN_ASP Sphere_d::get_surface_w_direction_constraints( const Eigen::MatrixXd& constraintNorms ){
	// Get a mesh consisting of all the triangles for which all vertices with rays center--to->V_i that do not violate constraintNorms
	
	bool SHOWDEBUG = false , 
		 TRIDETAIL = false ; // if( SHOWDEBUG ){ cout << "" << endl; }
	
	if( SHOWDEBUG ){ cout << "Init" << endl
						  << "Got the following norms:" << endl
						  << constraintNorms << endl
						  << "Center: " << center << endl; }
	
	TriMeshVFN_ASP rtnMesh;
	rtnMesh.center = center; // The new mesh will have the same center
	rtnMesh.type = SPHERICAL; 
	size_t numTri  = F.rows();
	bool triGood   = true;
	Eigen::Vector3d Vray;
	Eigen::Vector3d V_ij;
	Eigen::Vector3i f_i;
	if( SHOWDEBUG ){ cout << "About to iterate ..." << endl; }
	// 1. For each triangle
	for( size_t i = 0 ; i < numTri ; i++ ){
		if( SHOWDEBUG ){ cout << "Test triangle " << i+1 << " ..." << endl; }
		triGood = true;
		// 2. For each vertex
		for( size_t j = 0 ; j < 3 ; j++ ){
			// 3. Check if the vertex meets the constraints
			// A. Get ray
			V_ij = V.row( F(i,j) );
			Vray = ( V_ij - center ).normalized();
			if( SHOWDEBUG && TRIDETAIL ){ cout << "\tRay: " << Vray << endl
											   << "\t\tCenter: " << center << endl; }
			// B. Test
			if( !test_against_constraint_norms( constraintNorms , Vray ) ){  triGood = false;  }
		}
		// 4. If so
		if( triGood ){
			if( SHOWDEBUG ){ cout << "Add triangle " << i+1 << " ..." << endl; }
			f_i = Eigen::Vector3i( 0 , 0 , 0 );
			for( size_t j = 0 ; j < 3 ; j++ ){
				// 5. Add vertices
				V_ij = V.row( F(i,j) );
				if( SHOWDEBUG && TRIDETAIL ){ cout << "Fetched vertex: " << V_ij << endl; }
				rtnMesh.V = copy_V_plus_row( rtnMesh.V , V_ij ); // Extend vertices list by 1 R3 vector , return copy
				// 6. Add facet
				f_i( j ) = rtnMesh.V.rows() - 1;
			}
			rtnMesh.F = copy_F_plus_row( rtnMesh.F , f_i );
		}
	}
	
	if( SHOWDEBUG ){ cout << "About to calc normals ..." << endl; }
	// 7. return mesh
	rtnMesh.N = N_from_VF( rtnMesh.V , rtnMesh.F );
	if( SHOWDEBUG ){ cout << "Return ..." << endl; }
	return rtnMesh;
}

TriMeshVFN_ASP Sphere_d::get_surface_w_planar_constraints( const Eigen::MatrixXd& constraintNorms , const Eigen::MatrixXd& constraintPnts ){
	// Get a mesh consisting of all the triangles for which all vertices with rays center--to->V_i that do not violate constraintNorms
	TriMeshVFN_ASP rtnMesh;
	rtnMesh.center = center; // The new mesh will have the same center
	rtnMesh.type   = SPHERICAL;
	size_t numTri  = F.rows();
	bool triGood   = true;
	Eigen::Vector3d Vray;
	Eigen::Vector3d V_ij;
	Eigen::Vector3i f_i;
	// 1. For each triangle
	for( size_t i = 0 ; i < numTri ; i++ ){
		triGood = true;
		// 2. For each vertex
		for( size_t j = 0 ; j < 3 ; j++ ){
			// 3. Check if the vertex meets the constraints
			// A. Get ray
			V_ij = V.row( F(i,j) );
			//~ Vray = ( V_ij - center ).normalized();
			// B. Test
			if( !test_against_constraint_planes( constraintNorms , constraintPnts , V_ij ) ){  triGood = false;  }
		}
		// 4. If so
		if( triGood ){
			f_i = Eigen::Vector3i( 0 , 0 , 0 );
			for( size_t j = 0 ; j < 3 ; j++ ){
				// 5. Add vertices
				V_ij = V.row( F(i,j) );
				rtnMesh.V = copy_V_plus_row( rtnMesh.V , V_ij ); // Extend vertices list by 1 R3 vector , return copy
				// 6. Add facet
				f_i( j ) = rtnMesh.V.rows() - 1;
			}
			rtnMesh.F = copy_F_plus_row( rtnMesh.F , f_i );
		}
	}
	// 7. return mesh
	rtnMesh.N = N_from_VF( rtnMesh.V , rtnMesh.F );
	return rtnMesh;
}

// ~ Direction Query ~
Eigen::MatrixXd Sphere_d::sample_directions_from_direction_constraints( const Eigen::MatrixXd& constraintNorms ){
	// Get a mesh consisting of all the triangles for which all vertices with rays center--to->V_i that do not violate constraintNorms
	
	bool SHOWDEBUG = false , 
		 TRIDETAIL = false ; // if( SHOWDEBUG ){ cout << "" << endl; }
	
	if( SHOWDEBUG ){ cout << "Init" << endl
						  << "Got the following norms:" << endl
						  << constraintNorms << endl
						  << "Center: " << center << endl; }
	
	Eigen::MatrixXd rtnDirs;
	Eigen::Vector3d Vray;
	Eigen::Vector3d V_i;
	size_t /* -- */ numV = V.rows();
	if( SHOWDEBUG ){ cout << "About to iterate ..." << endl; }
	// 1. For each vertex
	for( size_t i = 0 ; i < numV ; i++ ){
		V_i = V.row(i);
		Vray = ( V_i - center ).normalized();
		if( SHOWDEBUG ){ cout << "Test direction " << Vray << " ..." << endl; }
		if( test_against_constraint_norms( constraintNorms , Vray ) ){  rtnDirs = copy_V_plus_row( rtnDirs , Vray );  }
	}
	return rtnDirs;
}

Eigen::MatrixXd Sphere_d::get_all_spherical_directions(){
	// Get a mesh consisting of all the triangles for which all vertices with rays center--to->V_i 
	
	bool SHOWDEBUG = false , 
		 TRIDETAIL = false ; // if( SHOWDEBUG ){ cout << "" << endl; }
	
	if( SHOWDEBUG ){ cout << "Init" << endl
						  << "Center: " << center << endl; }
	
	Eigen::MatrixXd rtnDirs;
	Eigen::Vector3d Vray;
	Eigen::Vector3d V_i;
	size_t /* -- */ numV = V.rows();
	if( SHOWDEBUG ){ cout << "About to iterate ..." << endl; }
	// 1. For each vertex
	for( size_t i = 0 ; i < numV ; i++ ){
		V_i = V.row(i);
		Vray = ( V_i - center ).normalized();
		rtnDirs = copy_V_plus_row( rtnDirs , Vray );
	}
	return rtnDirs;
}

// __ End Sphere_d __


// == Special Meshes ==


TriMeshVFN_ASP revolved_segment( const Eigen::Vector3d& center , const Eigen::Vector3d& axis , double radius , double height , size_t divisions ){
	// Return a mesh that represents the surface of revolution of line segment with 'height' that is parallel to the axis of revolution at 'radius'
	
	bool SHOWDEBUG = false; // if( SHOWDEBUG ){  cout << "" << endl;  }
	
	TriMeshVFN_ASP rtnMesh;
	rtnMesh.center = center;
	rtnMesh.axis   = axis;
	rtnMesh.type   = REVOLUTE;
	rtnMesh.V      = Eigen::MatrixXd::Zero( divisions * 6 , 3 );
	rtnMesh.F      = Eigen::MatrixXi::Zero( divisions * 2 , 3 );
	Eigen::Vector3d measureBgn = axis.cross( Eigen::Vector3d( 1 , 2 , 3 ) );
	Eigen::Vector3d top_V0;
	Eigen::Vector3d top_V1;
	Eigen::Vector3d top_V2;
	Eigen::Vector3d btm_V0;
	Eigen::Vector3d btm_V1;
	Eigen::Vector3d btm_V2;
	// 1. Get circles
	Eigen::MatrixXd topCirc = circle_arc_3D( axis , center + axis.normalized() * (  height/2.0 ) , 
											 radius , measureBgn , 2 * M_PI , divisions );
	Eigen::MatrixXd btmCirc = circle_arc_3D( axis , center + axis.normalized() * ( -height/2.0 ) , 
											 radius , measureBgn , 2 * M_PI , divisions );
	size_t ip1 ;
	// 2. For each pair of cicle points 
	for( size_t i = 0 ; i < divisions ; i++ ){
		ip1 = wrap_index_for_len( i+1 , divisions );
		if( SHOWDEBUG ){  cout << "i: " << i << " i , i+1: " << ip1 << endl;  }
		// 3. Create two triangles
		// A. Top
		top_V0 = topCirc.row( i   );
		top_V1 = btmCirc.row( ip1 );
		top_V2 = topCirc.row( ip1 );
		if( SHOWDEBUG ){  cout << "Top Triangle: " << top_V0 << endl << top_V1 << endl << top_V2 << endl;  }
		// B. Bottom
		btm_V0 = topCirc.row( i   );
		btm_V1 = btmCirc.row( i   );
		btm_V2 = btmCirc.row( ip1 );
		if( SHOWDEBUG ){  cout << "Bottom Triangle: " << btm_V0 << endl << btm_V1 << endl << btm_V2 << endl;  }
		// 4. Add triangles to the mesh
		// A. Top
		rtnMesh.V.row( i * 6 + 0 ) = top_V0;
		rtnMesh.V.row( i * 6 + 1 ) = top_V1;
		rtnMesh.V.row( i * 6 + 2 ) = top_V2;
		rtnMesh.F.row( i * 2 + 0 ) = Eigen::Vector3i( i*6+0 , i*6+1 , i*6+2 );
		// B. Bottom
		rtnMesh.V.row( i * 6 + 3 ) = btm_V0;
		rtnMesh.V.row( i * 6 + 4 ) = btm_V1;
		rtnMesh.V.row( i * 6 + 5 ) = btm_V2;
		rtnMesh.F.row( i * 2 + 1 ) = Eigen::Vector3i( i*6+3 , i*6+4 , i*6+5 );
	}
	// 5. Calc normals and return mesh
	rtnMesh.N = N_from_VF( rtnMesh.V , rtnMesh.F );
	return rtnMesh;
}

// __ End Special __


TriMeshVFN_ASP mesh_shadow_along_Z( const TriMeshVFN_ASP& original , double squareSide ,
									const Eigen::Vector3d& origin ,
									const Eigen::Vector3d& xBasis , const Eigen::Vector3d& yBasis , const Eigen::Vector3d& zBasis ){
										
	bool SHOWDEBUG = false; // if( SHOWDEBUG ){  cout << "" << endl;  }
										
	if( SHOWDEBUG ){  cout << "Init ..." << endl;  }
										
	TriMeshVFN_ASP rtnMesh;
	rtnMesh.center = origin;
	rtnMesh.axis   = zBasis;
	rtnMesh.type   = SHADOW;
	// 1. Get the AABB specified bases
	Eigen::MatrixXd arBB = arbitrary_BB_parent_frame( original.V ,
													  origin , 
													  xBasis , yBasis , zBasis );
	if( SHOWDEBUG ){  cout << "Bounding Box: " << endl <<  arBB << endl;  }
	// 2. Get the bounding X-Y rectangle
	Eigen::Vector3d crnr1 = arBB.row(0);
					crnr1 = pnt_proj_to_plane( crnr1 , origin , zBasis );  if( SHOWDEBUG ){  cout << "Corner 1: " << crnr1 << endl;  }
	Eigen::Vector3d crnr2 = arBB.row(1);
					crnr2 = pnt_proj_to_plane( crnr2 , origin , zBasis );  if( SHOWDEBUG ){  cout << "Corner 2: " << crnr2 << endl;  }
	double xtent_X = xBasis.dot( crnr2 - crnr1 );  if( SHOWDEBUG ){  cout << "X extent: " << xtent_X << endl;  }
	double xtent_Y = yBasis.dot( crnr2 - crnr1 );  if( SHOWDEBUG ){  cout << "Y extent: " << xtent_Y << endl;  }
	double X = 0.0 ,
		   Y = 0.0 ;
	Eigen::Vector3d vA;  Eigen::Vector3d vB;  Eigen::Vector3d vC;  Eigen::Vector3d vD;  Eigen::Vector3d vE;  
	Eigen::MatrixXd t1 = Eigen::MatrixXd::Zero( 3 , 3 );
	Eigen::MatrixXd t2 = Eigen::MatrixXd::Zero( 3 , 3 );
	Eigen::MatrixXd t3 = Eigen::MatrixXd::Zero( 3 , 3 );
	Eigen::MatrixXd t4 = Eigen::MatrixXd::Zero( 3 , 3 );
	Eigen::MatrixXd triCntr;
	RayHits /* - */ hitResult;
	size_t nextVrow = 0;
	// 3. Tile the mesh with squares
	while( X <= xtent_X ){
		Y = 0.0;
		while( Y <= xtent_Y ){
			if( SHOWDEBUG ){  cout << "X: " << X << " , Y: " << Y << endl;  }
			// 4. Tile the squares with triangles
			vA = crnr1 + xBasis * X + yBasis * Y;
			vB = vA + xBasis * squareSide;
			vC = vA + xBasis * squareSide + yBasis * squareSide;
			vD = vA +                       yBasis * squareSide;
			vE = ( vA + vB + vC + vD ) / 4.0;
			t1.row(0) = vA;  t1.row(1) = vB;  t1.row(2) = vE;
			t2.row(0) = vB;  t2.row(1) = vC;  t2.row(2) = vE;
			t3.row(0) = vC;  t3.row(1) = vD;  t3.row(2) = vE;
			t4.row(0) = vD;  t4.row(1) = vA;  t4.row(2) = vE;
			if( SHOWDEBUG ){  cout << "\tvA: " << vA << endl
								   << "\tvB: " << vB << endl
								   << "\tvC: " << vC << endl
								   << "\tvD: " << vD << endl
								   << "\tvE: " << vE << endl;  }
			// 5. Get the triangle centers
			// 6. Shoot rays from the triangle centers
			// 7. For each collision, add the triangle to the mesh
			triCntr = get_average_V( t1 );
			hitResult = ray_intersect_VFN( ( triCntr - zBasis * max( xtent_X , xtent_Y ) * 4 ) , zBasis , original );
			if( hitResult.anyHits ){
				rtnMesh.V = vstack( rtnMesh.V , t1 );
				rtnMesh.F = copy_F_plus_row( rtnMesh.F , Eigen::Vector3i( nextVrow , nextVrow+1 , nextVrow+2 ) );
				nextVrow += 3;
			}
			triCntr = get_average_V( t2 );
			hitResult = ray_intersect_VFN( ( triCntr - zBasis * max( xtent_X , xtent_Y ) * 4 ) , zBasis , original );
			if( hitResult.anyHits ){
				rtnMesh.V = vstack( rtnMesh.V , t2 );
				rtnMesh.F = copy_F_plus_row( rtnMesh.F , Eigen::Vector3i( nextVrow , nextVrow+1 , nextVrow+2 ) );
				nextVrow += 3;
			}
			triCntr = get_average_V( t3 );
			hitResult = ray_intersect_VFN( ( triCntr - zBasis * max( xtent_X , xtent_Y ) * 4 ) , zBasis , original );
			if( hitResult.anyHits ){
				rtnMesh.V = vstack( rtnMesh.V , t3 );
				rtnMesh.F = copy_F_plus_row( rtnMesh.F , Eigen::Vector3i( nextVrow , nextVrow+1 , nextVrow+2 ) );
				nextVrow += 3;
			}
			triCntr = get_average_V( t4 );
			hitResult = ray_intersect_VFN( ( triCntr - zBasis * max( xtent_X , xtent_Y ) * 4 ) , zBasis , original );
			if( hitResult.anyHits ){
				rtnMesh.V = vstack( rtnMesh.V , t4 );
				rtnMesh.F = copy_F_plus_row( rtnMesh.F , Eigen::Vector3i( nextVrow , nextVrow+1 , nextVrow+2 ) );
				nextVrow += 3;
			}
			Y += squareSide;
		}
		X += squareSide;
	}
	// 8. Calc normals and return mesh
	rtnMesh.N = N_from_VF( rtnMesh.V , rtnMesh.F );
	if( SHOWDEBUG ){  cout << "Returning a mesh with " << rtnMesh.V.rows() << " points, " 
						   << rtnMesh.F.rows() << " facets, and " << rtnMesh.N.rows() << " normals." << endl;  }
	return rtnMesh;
}

TriMeshVFN_ASP get_mesh_w_direction_constraints( const TriMeshVFN_ASP& original , const Eigen::MatrixXd& constraintNorms ){
	// Get a mesh consisting of all the triangles for which all vertices with rays center--to->V_i that do not violate constraintNorms
	// NOTE: This function returns a dense mesh no matter what sort of mesh was passed to it
	// NOTE: This function assumes that 'original.center' has been set, and that it is the appropriate point from which to evaluate direction
	
	bool SHOWDEBUG = false; // if( SHOWDEBUG ){ cout << "" << endl; }
	
	if( SHOWDEBUG ){ cout << "Init" << endl; }
	
	TriMeshVFN_ASP rtnMesh;
	rtnMesh.center = original.center; // The new mesh will have the same center
	rtnMesh.axis   = original.axis; // - The new mesh will have the same axis
	rtnMesh.type   = original.type; // - The new mesh will have the same type
	size_t numTri  = original.F.rows();
	bool triGood   = true;
	Eigen::Vector3d Vray;
	Eigen::Vector3d V_ij;
	Eigen::Vector3i f_i;
	
	if( SHOWDEBUG ){ cout << "About to iterate ..." << endl; }
	// 1. For each triangle
	for( size_t i = 0 ; i < numTri ; i++ ){
		if( SHOWDEBUG ){ cout << "Test triangle " << i+1 << " ..." << endl; }
		triGood = true;
		// 2. For each vertex
		for( size_t j = 0 ; j < 3 ; j++ ){
			// 3. Check if the vertex meets the constraints
			// A. Get ray
			V_ij = original.V.row( original.F(i,j) );
			Vray = ( V_ij - original.center ).normalized();
			// B. Test
			if( !test_against_constraint_norms( constraintNorms , Vray ) ){  triGood = false;  }
		}
		// 4. If so
		if( triGood ){
			if( SHOWDEBUG ){ cout << "Add triangle " << i+1 << " ..." << endl; }
			f_i = Eigen::Vector3i( 0 , 0 , 0 );
			for( size_t j = 0 ; j < 3 ; j++ ){
				// 5. Add vertices
				V_ij = original.V.row( original.F(i,j) );
				if( SHOWDEBUG ){ cout << "Fetched vertex: " << V_ij << endl; }
				rtnMesh.V = copy_V_plus_row( rtnMesh.V , V_ij ); // Extend vertices list by 1 R3 vector , return copy
				if( SHOWDEBUG ){ cout << "Stored vertex:_ " << rtnMesh.V.row( rtnMesh.V.rows()-1 ) << endl; }
				// 6. Add facet
				f_i( j ) = rtnMesh.V.rows() - 1;
			}
			rtnMesh.F = copy_F_plus_row( rtnMesh.F , f_i );
		}
	}
	
	if( SHOWDEBUG ){ cout << "About to calc normals ..." << endl; }
	// 7. return mesh
	rtnMesh.N = N_from_VF( rtnMesh.V , rtnMesh.F ); 
	if( SHOWDEBUG ){ cout << "Return ..." << endl; }
	return rtnMesh;
}


// == Mesh Collision ==

double V_component_selector( Eigen::Vector3d D , Eigen::Vector3d Vi_i ){
	// Return the component of 'Vi_i' indicated by Moller 1997 , Otherwise return NaN
	double maxD = max( max( abs( D(0) ) , abs( D(1) ) ) , abs( D(2) ) ); 
	if( abs( D(0) ) == maxD ) return Vi_i(0);
	if( abs( D(1) ) == maxD ) return Vi_i(1);
	if( abs( D(2) ) == maxD ) return Vi_i(2);
	return nan("");
}

bool triangle_collision( Eigen::Vector3d V1_0 , Eigen::Vector3d V1_1 , Eigen::Vector3d V1_2 , Eigen::Vector3d norm1 ,
						 Eigen::Vector3d V2_0 , Eigen::Vector3d V2_1 , Eigen::Vector3d V2_2 , Eigen::Vector3d norm2 ){
	// Return true if two triangles intersect , Otherwise return false , Moller 1997
	// NOTE: At this time, ignoring: { point contact , edge contact , co-planar }
	
	// 1. Compute plane equation of triangle 2.
	double d_2 = -norm2.dot( V2_0 );
	// 2. Reject as trivial if all points of triangle 1 are on same side.
	double d_V1_0 = norm2.dot( V1_0 ) + d_2;   if( abs( d_V1_0 ) < EPSILON ) d_V1_0 = 0.0;
	double d_V1_1 = norm2.dot( V1_1 ) + d_2;   if( abs( d_V1_1 ) < EPSILON ) d_V1_1 = 0.0;
	double d_V1_2 = norm2.dot( V1_2 ) + d_2;   if( abs( d_V1_2 ) < EPSILON ) d_V1_2 = 0.0;
	if( ( d_V1_0 > 0 && d_V1_1 > 0 && d_V1_2 > 0 ) || ( d_V1_0 < 0 && d_V1_1 < 0 && d_V1_2 < 0 ) ){ return false; }
	// 3. Compute plane equation of triangle 1.
	double d_1 = -norm1.dot( V1_0 );
	// 4. Reject as trivial if all points of triangle 2 are on same side.
	double d_V2_0 = norm1.dot( V2_0 ) + d_1;   if( abs( d_V2_0 ) < EPSILON ) d_V2_0 = 0.0;
	double d_V2_1 = norm1.dot( V2_1 ) + d_1;   if( abs( d_V2_1 ) < EPSILON ) d_V2_1 = 0.0;
	double d_V2_2 = norm1.dot( V2_2 ) + d_1;   if( abs( d_V2_2 ) < EPSILON ) d_V2_2 = 0.0;
	if( ( d_V2_0 >= 0 && d_V2_1 >= 0 && d_V2_2 >= 0 ) || ( d_V2_0 <= 0 && d_V2_1 <= 0 && d_V2_2 <= 0 ) ){ return false; }
	// 5. Compute intersection line and project onto largest axis.
	Eigen::Vector3d D = norm1.cross( norm2 );
	// 6. Compute the intervals for each triangle.
	double p_V1_0 = V_component_selector( D , V1_0 );
	double p_V1_1 = V_component_selector( D , V1_1 );
	double p_V1_2 = V_component_selector( D , V1_2 );
	double p_V2_0 = V_component_selector( D , V2_0 );
	double p_V2_1 = V_component_selector( D , V2_1 );
	double p_V2_2 = V_component_selector( D , V2_2 );
	
	double t1_1   = p_V1_0 + ( p_V1_1 - p_V1_0 ) * ( d_V1_0 / ( d_V1_0 - d_V1_1 ) );
	double t1_2   = p_V1_0 + ( p_V1_2 - p_V1_0 ) * ( d_V1_0 / ( d_V1_0 - d_V1_2 ) );
	
	double t2_1   = p_V2_0 + ( p_V2_1 - p_V2_0 ) * ( d_V2_0 / ( d_V2_0 - d_V2_1 ) );
	double t2_2   = p_V2_0 + ( p_V2_2 - p_V2_0 ) * ( d_V2_0 / ( d_V2_0 - d_V2_2 ) );
	
	// 7. Intersect the intervals.
	return
		// One of the T1 interval endpoints is within T2 interval
		( ( t1_1 < t2_1 ) && ( t1_1 > t2_2 ) ) ||
		( ( t1_1 > t2_1 ) && ( t1_1 < t2_2 ) ) ||
		( ( t1_2 < t2_1 ) && ( t1_2 > t2_2 ) ) ||
		( ( t1_2 > t2_1 ) && ( t1_2 < t2_2 ) ) || 
		// One of the T2 interval endpoints is within T1 interval
		( ( t2_1 < t1_1 ) && ( t2_1 > t1_2 ) ) ||
		( ( t2_1 > t1_1 ) && ( t2_1 < t1_2 ) ) ||
		( ( t2_2 < t1_1 ) && ( t2_2 > t1_2 ) ) ||
		( ( t2_2 > t1_1 ) && ( t2_2 < t1_2 ) )
	;
}

// __ End Collision __

// ___ End Mesh ___
