////////////////////////////////////////////////////////////////////////
// Class:       TestInference
// Plugin Type: analyzer (Unknown Unknown)
// File:        TestInference_module.cc
//
// Generated at Thu Oct 26 08:47:26 2023 by Giuseppe Cerati using cetskelgen
// from  version .
////////////////////////////////////////////////////////////////////////

#include "art/Framework/Core/EDAnalyzer.h"
#include "art/Framework/Core/ModuleMacros.h"
#include "art/Framework/Principal/Event.h"
#include "art/Framework/Principal/Handle.h"
#include "art/Framework/Principal/Run.h"
#include "art/Framework/Principal/SubRun.h"
#include "canvas/Utilities/InputTag.h"
#include "fhiclcpp/ParameterSet.h"
#include "messagefacility/MessageLogger/MessageLogger.h"
#include "canvas/Persistency/Common/FindManyP.h"

#include <array>
#include "lardataobj/RecoBase/Hit.h"
#include "lardataobj/RecoBase/SpacePoint.h"

#include "delaunator.hpp"

#include <torch/script.h>
#include <torchscatter/scatter.h>
//#include <torchsparse/sparse.h>

using std::array;
using std::vector;
using recob::Hit;
using recob::SpacePoint;

class TestInference;

class TestInference : public art::EDAnalyzer {
public:
  explicit TestInference(fhicl::ParameterSet const& p);
  // The compiler-generated destructor is fine for non-base
  // classes without bare pointers or other resource use.

  // Plugins should not be copied or assigned.
  TestInference(TestInference const&) = delete;
  TestInference(TestInference&&) = delete;
  TestInference& operator=(TestInference const&) = delete;
  TestInference& operator=(TestInference&&) = delete;

  // Required functions.
  void analyze(art::Event const& e) override;

private:

  // Declare member data here.

};


TestInference::TestInference(fhicl::ParameterSet const& p)
  : EDAnalyzer{p}  // ,
  // More initializers here.
{
  // Call appropriate consumes<>() for any products to be retrieved by this module.
}

void TestInference::analyze(art::Event const& e) 
{
  // Implementation of required member function here.
  // Get hits from the event record
  art::Handle< vector< Hit > > hitListHandle;
  vector< art::Ptr< Hit > > hitlist;
  if (e.getByLabel("nuslhits", hitListHandle)) {
    art::fill_ptr_vector(hitlist, hitListHandle);
  }

  std::cout << "Hits size=" << hitlist.size() << std::endl;
  if (hitlist.size()<10) return;

  /*
    wire, time, integral, rms
    norm p= u tensor([[389.0063, 173.4202, 144.4207,   4.5582],
                      [148.0289,  78.8351, 223.8940,   2.2621]])
    norm p= v tensor([[3.6914e+02, 1.7348e+02, 8.5748e+08, 4.4525e+00],
                      [1.4525e+02, 8.1396e+01, 1.0625e+13, 1.9224e+00]])
    norm p= y tensor([[547.3900, 173.1302, 109.5769,   4.1025],
                      [284.2066,  74.4782, 108.9379,   1.4318]])
   */
  vector<std::array<double, 4> > avgs = {std::array<double, 4>{389.0063, 173.4202, 144.4207,   4.5582},
                                         std::array<double, 4>{3.6914e+02, 1.7348e+02, 8.5748e+08, 4.4525e+00},
                                         std::array<double, 4>{547.3900, 173.1302, 109.5769,   4.1025}  };
  vector<std::array<double, 4> > devs = {std::array<double, 4>{148.0289,  78.8351, 223.8940,   2.2621},
                                         std::array<double, 4>{1.4525e+02, 8.1396e+01, 1.0625e+13, 1.9224e+00},
                                         std::array<double, 4>{284.2066,  74.4782, 108.9379,   1.4318}  };

  vector<vector<double> > nodeft(3,vector<double>());
  vector<vector<double> > coords(3,vector<double>());
  vector<vector<size_t> > idsmap(3,vector<size_t>());
  vector<size_t> idsmapRev(hitlist.size(),hitlist.size());
  for (auto h : hitlist) {
    idsmap[h->View()].push_back(h.key());
    idsmapRev[h.key()] = idsmap[h->View()].size()-1;
    coords[h->View()].push_back(h->PeakTime()*0.055);
    coords[h->View()].push_back(h->WireID().Wire*0.3);
    nodeft[h->View()].push_back( (h->WireID().Wire*0.3 - avgs[h->View()][0]) / devs[h->View()][0] );
    nodeft[h->View()].push_back( (h->PeakTime()*0.055  - avgs[h->View()][1]) / devs[h->View()][1] );
    nodeft[h->View()].push_back( (h->Integral()        - avgs[h->View()][2]) / devs[h->View()][2] );
    nodeft[h->View()].push_back( (h->RMS()             - avgs[h->View()][3]) / devs[h->View()][3] );
  }

  struct Edge {
    size_t n1; 
    size_t n2; 
    bool operator==(Edge& other) const {
      if ( this->n1==other.n1 && this->n2==other.n2 ) return true; 
      else return false;
    }; 
  };
  vector<vector<Edge> > edge2d(3,vector<Edge>());
  for (size_t p=0; p<3; p++) {
    std::cout << "Plane " << p << " has N hits=" << coords[p].size()/2 << std::endl;
    if (coords[p].size()/2<3) continue;
    delaunator::Delaunator d(coords[p]);
    std::cout << "Found N triangles=" << d.triangles.size()/3 << std::endl;
    for(std::size_t i = 0; i < d.triangles.size(); i+=3) {
      //std::cout << "Triangle bewteen points with ids=" << d.triangles[i] << " " << d.triangles[i + 1] << " " << d.triangles[i + 2] << std::endl;
      //create edges in both directions
      Edge e;
      e.n1 = d.triangles[i];
      e.n2 = d.triangles[i + 1];
      edge2d[p].push_back(e);
      e.n1 = d.triangles[i + 1];
      e.n2 = d.triangles[i];
      edge2d[p].push_back(e);
      //
      e.n1 = d.triangles[i];
      e.n2 = d.triangles[i + 2];
      edge2d[p].push_back(e);
      e.n1 = d.triangles[i + 2];
      e.n2 = d.triangles[i];
      edge2d[p].push_back(e);
      //
      e.n1 = d.triangles[i + 1];
      e.n2 = d.triangles[i + 2];
      edge2d[p].push_back(e);
      e.n1 = d.triangles[i + 2];
      e.n2 = d.triangles[i + 1];
      edge2d[p].push_back(e);
      //
    }
    //sort and cleanup duplicates
    std::sort(edge2d[p].begin(),edge2d[p].end(),[](const auto& i, const auto& j){return (i.n1!=j.n1 ? i.n1 < j.n1 : i.n2 < j.n2);});
    edge2d[p].erase( std::unique( edge2d[p].begin(), edge2d[p].end() ), edge2d[p].end() );
  }
  // vector<vector<size_t> > edge_idx_plane(3);
  // for (size_t p=0; p<3; p++) {
  //   edge_idx_plane[p] = vector<size_t>(edgen1[p].begin(),edgen1[p].end());
  //   edge_idx_plane[p].insert(edge_idx_plane[p].end()-1, edgen2[p].begin(),edgen2[p].end());
  // }

  // Get spacepoints from the event record
  art::Handle< vector< SpacePoint > > spListHandle;
  vector< art::Ptr< SpacePoint > > splist;
  if (e.getByLabel("sps", spListHandle)) {
      art::fill_ptr_vector(splist, spListHandle);
  }
  // Get assocations from spacepoints to hits
  vector< vector< art::Ptr< Hit > > > sp2Hit(splist.size());
  if (splist.size()>0) {
    art::FindManyP< Hit > fmp(spListHandle, e, "sps");
    for (size_t spIdx = 0; spIdx < sp2Hit.size(); ++spIdx) {
      sp2Hit[spIdx] = fmp.at(spIdx);
    }
  }

  //Edges are the same as in pyg, but order is not identical.
  //It should not matter but better verify that output is indeed the same.
  vector<vector<Edge> > edge3d(3,vector<Edge>());
  for (size_t i = 0; i < splist.size(); ++i) {
    for (size_t j = 0; j < sp2Hit[i].size(); ++j) {
      Edge e;
      e.n1 = idsmapRev[sp2Hit[i][j].key()];
      e.n2 = i;
      edge3d[sp2Hit[i][j]->View()].push_back(e);
    }
  }

  // //torch.Size([2, N]) first dimension is the plane hit id, second is the sp id
  // vector<std::vector<size_t> > edge_idx_nexus(3);
  // for (size_t p=0; p<3; p++) {
  //   edge_idx_nexus[p] = vector<size_t>(edgen2D[p].begin(),edgen2D[p].end());
  //   edge_idx_nexus[p].insert(edge_idx_nexus[p].end()-1, edgen3D[p].begin(),edgen3D[p].end());
  // }

  std::string planes[3] = {"u","v","y"};

  auto x = torch::Dict<std::string, torch::Tensor>();
  auto batch = torch::Dict<std::string, torch::Tensor>();
  for (size_t p=0;p<3;p++) {
    std::cout << "plane=" << p << std::endl;
    long int dim = nodeft[p].size()/4;
    torch::Tensor ix = torch::zeros({dim,4},torch::dtype(torch::kFloat32));
    for (size_t n=0;n<nodeft[p].size();n=n+4) {
      std::cout << nodeft[p][n] << " " << nodeft[p][n+1] << " " << nodeft[p][n+2] << " " << nodeft[p][n+3] << " " << std::endl;
      ix[n/4][0] = nodeft[p][n];
      ix[n/4][1] = nodeft[p][n+1];
      ix[n/4][2] = nodeft[p][n+2];
      ix[n/4][3] = nodeft[p][n+3];
    }
    x.insert(planes[p],ix);
    torch::Tensor ib = torch::zeros({dim},torch::dtype(torch::kInt64));
    batch.insert(planes[p],ib);
  }

  auto edge_index_plane = torch::Dict<std::string, torch::Tensor>();
  for (size_t p=0;p<3;p++) {
    std::cout << "plane=" << p << std::endl;
    std::cout << "2d edge size=" << edge2d[p].size() << std::endl;
    for (size_t n=0;n<edge2d[p].size();n++) {
      std::cout << edge2d[p][n].n1 << " ";
    }
    std::cout << std::endl;
    for (size_t n=0;n<edge2d[p].size();n++) {
      std::cout << edge2d[p][n].n2 << " ";
    }
    long int dim = edge2d[p].size();
    torch::Tensor ix = torch::zeros({2,dim},torch::dtype(torch::kInt64));
    for (size_t n=0;n<edge2d[p].size();n++) {
      ix[0][n] = int(edge2d[p][n].n1);
      ix[1][n] = int(edge2d[p][n].n2);
    }
    edge_index_plane.insert(planes[p],ix);
    std::cout << std::endl;
  }

  auto edge_index_nexus = torch::Dict<std::string, torch::Tensor>();
  for (size_t p=0;p<3;p++) {
    std::cout << "plane=" << p << std::endl;
    std::cout << "3d edge size=" << edge3d[p].size() << std::endl;
    for (size_t n=0;n<edge3d[p].size();n++) {
      std::cout << edge3d[p][n].n1 << " ";
    }
    std::cout << std::endl;
    for (size_t n=0;n<edge3d[p].size();n++) {
      std::cout << edge3d[p][n].n2 << " ";
    }
    long int dim = edge3d[p].size();
    torch::Tensor ix = torch::zeros({2,dim},torch::dtype(torch::kInt64));
    for (size_t n=0;n<edge3d[p].size();n++) {
      ix[0][n] = int(edge3d[p][n].n1);
      ix[1][n] = int(edge3d[p][n].n2);
    }
    edge_index_nexus.insert(planes[p],ix);
    std::cout << std::endl;
  }  

  long int spdim = splist.size();
  auto nexus = torch::empty({spdim,0},torch::dtype(torch::kInt64));

  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(x);
  inputs.push_back(edge_index_plane);
  inputs.push_back(edge_index_nexus);
  inputs.push_back(nexus);
  inputs.push_back(batch);
  torch::jit::script::Module module = torch::jit::load("model.pt");
  std::cout << "FORWARD!" << std::endl;
  auto outputs = module.forward(inputs).toGenericDict();
  std::cout << "output =" << outputs << std::endl;
}

DEFINE_ART_MODULE(TestInference)
