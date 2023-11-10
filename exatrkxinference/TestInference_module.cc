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
  bool debug = 0;

  // Implementation of required member function here.
  // Get hits from the event record
  art::Handle< vector< Hit > > hitListHandle;
  vector< art::Ptr< Hit > > hitlist;
  if (e.getByLabel("nuslhits", hitListHandle)) {
    art::fill_ptr_vector(hitlist, hitListHandle);
  }

  if (debug) std::cout << "Hits size=" << hitlist.size() << std::endl;
  if (hitlist.size()<10) return;

  /*
    normalization factors
    wire, time, integral, rms
    u [389.00632   173.42017   144.42065     4.5582113]
      [148.02893    78.83508   223.89404     2.2621224]
    v [3.6914261e+02 1.7347592e+02 8.5748262e+08 4.4525051e+00]
      [1.4524565e+02 8.1395981e+01 1.0625440e+13 1.9223815e+00]
    y [547.38995   173.13017   109.57691     4.1024675]
      [284.20657    74.47823   108.93791     1.4318414]
   */
  vector<std::array<float, 4> > avgs = {std::array<float, 4>{389.00632, 173.42017, 144.42065, 4.5582113},
					std::array<float, 4>{3.6914261e+02, 1.7347592e+02, 8.5748262e+08, 4.4525051e+00},
					std::array<float, 4>{547.38995, 173.13017, 109.57691, 4.1024675}  };
  vector<std::array<float, 4> > devs = {std::array<float, 4>{148.02893, 78.83508, 223.89404, 2.2621224},
					std::array<float, 4>{1.4524565e+02, 8.1395981e+01, 1.0625440e+13, 1.9223815e+00},
					std::array<float, 4>{284.20657, 74.47823, 108.93791, 1.4318414}  };

  vector<vector<float> > nodeft_bare(3,vector<float>());
  vector<vector<float> > nodeft(3,vector<float>());
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
    nodeft_bare[h->View()].push_back( h->WireID().Wire*0.3 );
    nodeft_bare[h->View()].push_back( h->PeakTime()*0.055 );
    nodeft_bare[h->View()].push_back( h->Integral() );
    nodeft_bare[h->View()].push_back( h->RMS() );
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
    if (debug) std::cout << "Plane " << p << " has N hits=" << coords[p].size()/2 << std::endl;
    if (coords[p].size()/2<3) continue;
    delaunator::Delaunator d(coords[p]);
    if (debug) std::cout << "Found N triangles=" << d.triangles.size()/3 << std::endl;
    for(std::size_t i = 0; i < d.triangles.size(); i+=3) {
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

  std::string planes[3] = {"u","v","y"};

  auto x = torch::Dict<std::string, torch::Tensor>();
  auto batch = torch::Dict<std::string, torch::Tensor>();
  for (size_t p=0;p<3;p++) {
    if (debug) std::cout << "plane=" << p << std::endl;
    long int dim = nodeft[p].size()/4;
    torch::Tensor ix = torch::zeros({dim,4},torch::dtype(torch::kFloat32));
    if (debug) std::cout << std::fixed;
    if (debug) std::cout << std::setprecision(4);
    if (debug) std::cout << "before, plane=" << planes[p] << std::endl;
    for (size_t n=0;n<nodeft_bare[p].size();n=n+4) {
      if (debug) std::cout << nodeft_bare[p][n] << " " << nodeft_bare[p][n+1] << " " << nodeft_bare[p][n+2] << " " << nodeft_bare[p][n+3] << " " << std::endl;
    }
    if (debug) std::cout << std::scientific;
    if (debug) std::cout << "after, plane=" << planes[p] << std::endl;
    for (size_t n=0;n<nodeft[p].size();n=n+4) {
      if (debug) std::cout << nodeft[p][n] << " " << nodeft[p][n+1] << " " << nodeft[p][n+2] << " " << nodeft[p][n+3] << " " << std::endl;
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
    if (debug) std::cout << "plane=" << p << std::endl;
    if (debug) std::cout << "2d edge size=" << edge2d[p].size() << std::endl;
    for (size_t n=0;n<edge2d[p].size();n++) {
      if (debug) std::cout << edge2d[p][n].n1 << " ";
    }
    if (debug) std::cout << std::endl;
    for (size_t n=0;n<edge2d[p].size();n++) {
      if (debug) std::cout << edge2d[p][n].n2 << " ";
    }
    long int dim = edge2d[p].size();
    torch::Tensor ix = torch::zeros({2,dim},torch::dtype(torch::kInt64));
    for (size_t n=0;n<edge2d[p].size();n++) {
      ix[0][n] = int(edge2d[p][n].n1);
      ix[1][n] = int(edge2d[p][n].n2);
    }
    edge_index_plane.insert(planes[p],ix);
    if (debug) std::cout << std::endl;
  }

  auto edge_index_nexus = torch::Dict<std::string, torch::Tensor>();
  for (size_t p=0;p<3;p++) {
    if (debug) std::cout << "plane=" << p << std::endl;
    if (debug) std::cout << "3d edge size=" << edge3d[p].size() << std::endl;
    for (size_t n=0;n<edge3d[p].size();n++) {
      if (debug) std::cout << edge3d[p][n].n1 << " ";
    }
    if (debug) std::cout << std::endl;
    for (size_t n=0;n<edge3d[p].size();n++) {
      if (debug) std::cout << edge3d[p][n].n2 << " ";
    }
    long int dim = edge3d[p].size();
    torch::Tensor ix = torch::zeros({2,dim},torch::dtype(torch::kInt64));
    for (size_t n=0;n<edge3d[p].size();n++) {
      ix[0][n] = int(edge3d[p][n].n1);
      ix[1][n] = int(edge3d[p][n].n2);
    }
    edge_index_nexus.insert(planes[p],ix);
    if (debug) std::cout << std::endl;
  }

  long int spdim = splist.size();
  auto nexus = torch::empty({spdim,0},torch::dtype(torch::kFloat32));

  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(x);
  inputs.push_back(edge_index_plane);
  inputs.push_back(edge_index_nexus);
  inputs.push_back(nexus);
  inputs.push_back(batch);
  torch::jit::script::Module module = torch::jit::load("model.pt");
  if (debug) std::cout << "FORWARD!" << std::endl;
  auto outputs = module.forward(inputs).toGenericDict();
  if (debug) std::cout << "output =" << outputs << std::endl;
  auto s_u = outputs.at("x_semantic").toGenericDict().at("u").toTensor();
  auto s_v = outputs.at("x_semantic").toGenericDict().at("v").toTensor();
  auto s_y = outputs.at("x_semantic").toGenericDict().at("y").toTensor();
  for (int j=0;j<5;j++) {
    std::cout <<"x_semantic category=" << j << " : ";
    for (int i = 0; i < s_u.sizes()[0]; ++i) std::cout << s_u[i][j].item<float>() << ", ";
    for (int i = 0; i < s_v.sizes()[0]; ++i) std::cout << s_v[i][j].item<float>() << ", ";
    for (int i = 0; i < s_y.sizes()[0]; ++i) std::cout << s_y[i][j].item<float>() << ", ";
    std::cout << std::endl;
  }
  //
  auto f_u = outputs.at("x_filter").toGenericDict().at("u").toTensor();
  auto f_v = outputs.at("x_filter").toGenericDict().at("v").toTensor();
  auto f_y = outputs.at("x_filter").toGenericDict().at("y").toTensor();
    std::cout <<"x_filter : ";
  for (int i = 0; i < f_u.numel(); ++i) std::cout << f_u[i].item<float>() << ", ";
  for (int i = 0; i < f_v.numel(); ++i) std::cout << f_v[i].item<float>() << ", ";
  for (int i = 0; i < f_y.numel(); ++i) std::cout << f_y[i].item<float>() << ", ";
  std::cout << std::endl;
}

DEFINE_ART_MODULE(TestInference)
