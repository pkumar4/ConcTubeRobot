#include <nanoflann.hpp>
#include <ctime>
#include <cstdlib>
#include <iostream>

#include <vtkPolyData.h>
#include <vtkSTLReader.h>
#include <vtkOBJReader.h>

#include <vtkSmartPointer.h>

#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

using namespace std;
using namespace nanoflann;

// This is a custom data set class
template <typename T>
struct PointCloud
{
	struct Point
	{
		T  x,y,z;
	};

	std::vector<Point>  pts;

	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return pts.size(); }

	// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
	inline T kdtree_distance(const T *p1, const size_t idx_p2,size_t /*size*/) const
	{
		const T d0=p1[0]-pts[idx_p2].x;
		const T d1=p1[1]-pts[idx_p2].y;
		const T d2=p1[2]-pts[idx_p2].z;
		return d0*d0+d1*d1+d2*d2;
	}

	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline T kdtree_get_pt(const size_t idx, int dim) const
	{
		if (dim==0) return pts[idx].x;
		else if (dim==1) return pts[idx].y;
		else return pts[idx].z;
	}

	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

};

template<typename T>
void generateAnatomyPointCloud(PointCloud<T> &point, const size_t N, vtkDataArray *dataArray,const T max_range = 10)
{

	
	std::cout << "Generating "<< N << " point cloud...";
	point.pts.resize(N);
	
	for (vtkIdType tupleIdx=0;tupleIdx<N;tupleIdx++)
	{	
		point.pts[tupleIdx].x = dataArray->GetComponent(tupleIdx, 0);
		point.pts[tupleIdx].y = dataArray->GetComponent(tupleIdx, 1);
		point.pts[tupleIdx].z = dataArray->GetComponent(tupleIdx, 2);
	}

	std::cout << "done\n";
}


template<typename num_t>
void kdtree_process(const size_t N,vtkDataArray *dataArray,string& ctr_inputFilename)
{
	PointCloud<num_t> cloud;

	// Generate points:
	generateAnatomyPointCloud(cloud, N,dataArray);

	// construct a kd-tree index:
	typedef KDTreeSingleIndexAdaptor<
		L2_Simple_Adaptor<num_t, PointCloud<num_t> > ,
		PointCloud<num_t>,
		3 /* dim */
		> my_kd_tree_t;

	my_kd_tree_t   index(3 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );
	index.buildIndex();


	const num_t query_pt[3] = { 0.5, 0.5, 0.5};

	vtkSmartPointer<vtkSTLReader> ctr_reader =
    vtkSmartPointer<vtkSTLReader>::New();
  	ctr_reader->SetFileName(ctr_inputFilename.c_str());
  	ctr_reader->Update();
	// Randomize Seed
	//srand(time(NULL));
	
	vtkSmartPointer<vtkPolyData> ctr_mesh = ctr_reader->GetOutput();
	vtkSmartPointer<vtkPoints> ctr_points = ctr_mesh->GetPoints();
	vtkSmartPointer<vtkDataArray> ctr_dataArray = ctr_points->GetData();
	
	std::vector<double> ctr_vector;
	vtkIdType ctr_numVectors = ctr_dataArray->GetNumberOfTuples();
	num_t ctr_array[ctr_numVectors] ;
	
	for (vtkIdType index=0;index<ctr_numVectors;index++){
		ctr_vector.push_back(*ctr_dataArray->GetTuple(index));
		ctr_array[index] = *ctr_dataArray->GetTuple(index);
		//cout<<" ctr_array["<<index<<"] = "<<ctr_array[index]<<"\n";
		//cout<<"*ctr_dataArray->GetTuple(index) "<<*ctr_dataArray->GetTuple(index);
	}
	
	// ----------------------------------------------------------------
	// radiusSearch():  Perform a search for the N closest points
	// ----------------------------------------------------------------
	{
		const num_t search_radius = static_cast<num_t>(0.0002);
		std::vector<std::pair<size_t,num_t> >   ret_matches;

		nanoflann::SearchParams params;
		
		const size_t nMatches = index.radiusSearch(&ctr_array[0],search_radius, ret_matches, params);
		
		cout << "radiusSearch(): radius=" << search_radius << " -> " << nMatches << " matches\n";
		for (size_t i=0;i<nMatches;i++)
			cout << "idx["<< i << "]=" << ret_matches[i].first << " dist["<< i << "]=" << ret_matches[i].second << endl;
		cout << "\n";


	}

}

int main(int argc, char *argv[])
{
	if ( argc < 3 )
    {
    cout << "Required parameters: Filename" << endl;
    return EXIT_FAILURE;
    }
    std::string inputFilename = argv[1];

	vtkSmartPointer<vtkOBJReader> anatomy_reader =
    vtkSmartPointer<vtkOBJReader>::New();
  	anatomy_reader->SetFileName(inputFilename.c_str());
  	anatomy_reader->Update();
	// Randomize Seed
	//srand(time(NULL));
	
	// Read anatomy structure
	vtkSmartPointer<vtkPolyData> mesh = anatomy_reader->GetOutput();
	vtkSmartPointer<vtkPoints> points = mesh->GetPoints();
	vtkSmartPointer<vtkDataArray> dataArray = points->GetData();
	vtkIdType numVectors = dataArray->GetNumberOfTuples();

	// Read robot structure
	std::string ctr_filename = argv[2];
    vtkSmartPointer<vtkSTLReader> ctr_reader =
  	vtkSmartPointer<vtkSTLReader>::New();
  	ctr_reader->SetFileName(ctr_filename.c_str());
  	ctr_reader->Update();
	
	
	kdtree_process<float>(numVectors,dataArray,ctr_filename);

	vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(anatomy_reader->GetOutputPort());

  vtkSmartPointer<vtkActor> anatomy_actor =
    vtkSmartPointer<vtkActor>::New();
  anatomy_actor->SetMapper(mapper);

  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  

  renderer->AddActor(anatomy_actor);
  

  vtkSmartPointer<vtkPolyDataMapper> ctr_mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  ctr_mapper->SetInputConnection(ctr_reader->GetOutputPort());

  vtkSmartPointer<vtkActor> ctr_actor =
    vtkSmartPointer<vtkActor>::New();
  ctr_actor->SetMapper(ctr_mapper);


	//kdtree_process<double>(100000);

	vtkSmartPointer<vtkActorCollection> actorCollection =  vtkSmartPointer<vtkActorCollection>::New();
	actorCollection->AddItem(ctr_actor);
	actorCollection->InitTraversal();
	for(vtkIdType i =0;i<actorCollection->GetNumberOfItems();i++){
		vtkActor* actor = actorCollection->GetNextActor();
		renderer->AddActor(actor);
	}

	vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
  renderer->SetBackground(.3, .6, .3);
  renderWindow->Render();
  renderWindowInteractor->Start();

	return 0;
}