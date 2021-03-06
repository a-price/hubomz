char ch_cap ( char c );
bool ch_eqi ( char c1, char c2 );
int ch_to_digit ( char c );
int i4_min ( int i1, int i2 );
double r8_max ( double x, double y );
double r8_min ( double x, double y );
double *r8vec_cross_3d ( double v1[3], double v2[3] );
double r8vec_dot ( int n, double a1[], double a2[] );
double r8vec_length ( int dim_num, double x[] );
bool s_eqi ( std::string s1, std::string s2 );
int s_len_trim ( std::string s );
double s_to_r8 ( std::string s, int *lchar, bool *error );
bool stla_check ( std::string input_file_name );
void stla_face_node_print ( int face_num, int face_node[] );
double *stla_face_normal_compute ( int node_num, int face_num, double node_xyz[], 
  int face_node[] );
void stla_face_normal_print ( int face_num, double face_normal[] );
void stla_node_xyz_print ( int node_num, double node_xyz[] );
int stla_offset_get ( void );
void stla_offset_set ( int offset );
bool stla_read ( std::string input_file_name, int node_num, int face_num,
  double node_xyz[], int face_node[], double face_normal[] );
void stla_size ( std::string input_file_name, int *solid_num, int *node_num,
  int *face_num, int *text_num );
void stla_size_print ( std::string input_file_name, int solid_num, int node_num,
  int face_num, int text_num );
void stla_write ( std::string output_file_name, int node_num, int face_num,
  double node_xyz[], int face_node[], double face_normal[] );
void timestamp ( void );
std::string word_next_read ( std::string s, bool *done );
//
//  This variable determines whether indices are 0 or 1 based.
//
static int stla_offset_value = 0;

