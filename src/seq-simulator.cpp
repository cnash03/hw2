#include "make_unique.h"
#include "world.h"
#include "quad-tree.h"
#include <algorithm>
#include <iostream>

// TASK 1

// NOTE: You may modify any of the contents of this file, but preserve all function types and names.
// You may add new functions if you believe they will be helpful.

const int QuadTreeLeafSize = 8;
class SequentialNBodySimulator : public INBodySimulator
{
public:
    std::shared_ptr<QuadTreeNode> buildQuadTree(std::vector<Particle> & particles, Vec2 bmin, Vec2 bmax)
    {
        auto node = std::make_shared<QuadTreeNode>();
        if (particles.size() <= QuadTreeLeafSize) {
            node->isLeaf = true;
            node->particles = particles;
            return node;
        }
        else{
            node->isLeaf = false;
            auto mid = (bmin + bmax) * 0.5f;
            //divide into 4 quad
            std::vector<Particle> q0, q1, q2, q3;
            for(auto p : particles) {
                if (p.position.y < mid.y) {     // Bottom
                    if (p.position.x < mid.x) q0.push_back(p); // Bottom-Left
                    else                      q1.push_back(p); // Bottom-Right
                } else {                        // Top
                    if (p.position.x < mid.x) q2.push_back(p); // Top-Left
                    else                      q3.push_back(p); // Top-Right
                }
            }

            // Assigning to indices 0-3 based on the code's expected spatial layout
            node->children[0] = buildQuadTree(q0, bmin, mid);
            node->children[1] = buildQuadTree(q1, Vec2(mid.x, bmin.y), Vec2(bmax.x, mid.y));
            node->children[2] = buildQuadTree(q2, Vec2(bmin.x, mid.y), Vec2(mid.x, bmax.y));
            node->children[3] = buildQuadTree(q3, mid, bmax);
        }
        return node;
    }
    virtual std::shared_ptr<AccelerationStructure> buildAccelerationStructure(std::vector<Particle> & particles)
    {
        // build quad-tree
        auto quadTree = std::make_shared<QuadTree>();

        // find bounds
        Vec2 bmin(1e30f, 1e30f);
        Vec2 bmax(-1e30f, -1e30f);

        for (auto & p : particles)
        {
            bmin.x = fminf(bmin.x, p.position.x);
            bmin.y = fminf(bmin.y, p.position.y);
            bmax.x = fmaxf(bmax.x, p.position.x);
            bmax.y = fmaxf(bmax.y, p.position.y);
        }

        quadTree->bmin = bmin;
        quadTree->bmax = bmax;

        // build nodes
        quadTree->root = buildQuadTree(particles, bmin, bmax);
        if (!quadTree->checkTree()) {
          std::cout << "Your Tree has Error!" << std::endl;
        }

        return quadTree;
    }
    virtual void simulateStep(AccelerationStructure * accel, std::vector<Particle> & particles, std::vector<Particle> & newParticles, StepParameters params) override
    {
        QuadTree* quadTree = static_cast<QuadTree*>(accel);        
        auto root = quadTree->root;
        auto min = quadTree->bmin;
        auto max = quadTree->bmax;
        
        for(int i =0 ; i<particles.size(); i++) {
            auto p = particles[i];
            std::vector<Particle> nearbyParticles;
            quadTree->getParticles(nearbyParticles, p.position, params.cullRadius);

            Vec2 force(0.0f, 0.0f);
            
            for(auto np: nearbyParticles){
                if (p.id == np.id) continue;

                force += computeForce(p, np, params.cullRadius);
            }
            newParticles[i] = updateParticle(p, force, params.deltaTime);
        }
    }
};

std::unique_ptr<INBodySimulator> createSequentialNBodySimulator()
{
    return std::make_unique<SequentialNBodySimulator>();
}
