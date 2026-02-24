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
        Vec2 mid = (bmin + bmax) * 0.5f;
        bool degenerate = (mid.x == bmin.x || mid.x == bmax.x || 
                            mid.y == bmin.y || mid.y == bmax.y); 
        if ((int)particles.size() <= QuadTreeLeafSize|| degenerate ) {
            node->isLeaf = true;
            node->particles = particles;
            return node;
        }
        else{
            node->isLeaf = false;

            std::vector<Particle> q0, q1, q2, q3;

            for (auto& p : particles)
            {
                bool left   = p.position.x < mid.x;
                bool bottom = p.position.y < mid.y;

                if (bottom && left)        q0.push_back(p);
                else if (bottom && !left)  q1.push_back(p);
                else if (!bottom && left)  q2.push_back(p);
                else                       q3.push_back(p);
            }

            // Assigning to indices 0-3 based on the code's expected spatial layout
            node->children[0] = buildQuadTree(q0,
                                            bmin,
                                            mid);

            node->children[1] = buildQuadTree(q1,
                                            Vec2(mid.x, bmin.y),
                                            Vec2(bmax.x, mid.y));

            node->children[2] = buildQuadTree(q2,
                                            Vec2(bmin.x, mid.y),
                                            Vec2(mid.x, bmax.y));

            node->children[3] = buildQuadTree(q3,
                                            mid,
                                            bmax);
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

        for (auto &p : particles)
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


        for (int i = 0; i < (int)particles.size(); i++)
        {
            auto &pi = particles[i];

            std::vector<Particle> nearbyParticles;
            accel->getParticles(nearbyParticles, pi.position, params.cullRadius);

            Vec2 force(0.0f, 0.0f);
            for (auto &pj : nearbyParticles)
            {
                if (pj.id == pi.id) continue;
                force += computeForce(pi, pj, params.cullRadius);
            }

            newParticles[i] = updateParticle(pi, force, params.deltaTime);
        }
    }
};

std::unique_ptr<INBodySimulator> createSequentialNBodySimulator()
{
    return std::make_unique<SequentialNBodySimulator>();
}
