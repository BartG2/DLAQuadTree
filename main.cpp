#include <chrono>
#include <random>
#include "raylib.h"
#include <iostream>
#include <vector>
#include <future>
#include <omp.h>
#include <array>
#include <algorithm>
#include <unordered_set>
#include <thread>
#include <functional>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <list>

std::mt19937 CreateGeneratorWithTimeSeed();
float RandomFloat(float min, float max, std::mt19937& rng);
bool Contains(const Rectangle& r1, const Rectangle& r2);

//-------------------------------------------------------------------------------------------------------------------------------------------------------

constexpr int screenWidth = 2560, screenHeight = 1440, numThreads = 2;
int maxTreeDepth = 5;
const float collisionThreshold = 1.1f, minimumStickDistance = 1.0f;
float stickingProbability = 1.0f;

std::mt19937 rng = CreateGeneratorWithTimeSeed();

//-------------------------------------------------------------------------------------------------------------------------------------------------------

class Particle {
public:
    Vector2 pos;
    Vector2 v;
    Vector2 a;
    Color color;
    bool isStuck;

    Particle(Vector2 position, Color col, Vector2 velocity = {0, 0}, Vector2 acceleration = {0, 0}){
        pos = position;
        v = velocity;
        a = acceleration;
        color = col;
        isStuck = false;
    }

    void RandomWalk(float stepSize, int numSteps) {
        if(!isStuck){
            for (int i = 0; i < numSteps; i++) {
                float dx = RandomFloat(-1, 1, rng);
                float dy = RandomFloat(-1, 1, rng);

                float newX = pos.x + dx * stepSize;
                float newY = pos.y + dy * stepSize;

                // Check if particle is out of bounds and correct position
                if (newX < 0) {
                    newX = 0;
                }
                else if (newX > screenWidth) {
                    newX = screenWidth;
                }
                if (newY < 0) {
                    newY = 0;
                }
                else if (newY > screenHeight) {
                    newY = screenHeight;
                }

                pos.x = newX;
                pos.y = newY;
            }
        }
    }

    void updatePosition(){
        pos.x += v.x;
        pos.y += v.y;

        v.x += a.x;
        v.y += a.y;
    }

};

class Timer{
public:
    std::chrono::time_point<std::chrono::high_resolution_clock> startPoint;

    Timer(){
        startPoint = std::chrono::high_resolution_clock::now();
    }

    ~Timer(){
        stop();
    }

    void stop(){
        auto endPoint = std::chrono::high_resolution_clock::now();
        auto start = std::chrono::time_point_cast<std::chrono::microseconds>(startPoint).time_since_epoch().count();
        auto end = std::chrono::time_point_cast<std::chrono::microseconds>(endPoint).time_since_epoch().count();

        auto duration = end - start;
        double ms = duration * 0.001;

        std::cout << "ms: " << ms << std::endl;
    }
};

class QuadTree{
public:

    int currentDepth;
    Rectangle currentSize;
    std::vector<Particle> particles;
    std::array<std::shared_ptr<QuadTree>, 4> children{};
    std::array<Rectangle, 4> childAreas{};

    QuadTree(const int setDepth, const Rectangle& setSize){
        currentDepth = setDepth;
        resize(setSize);
    }

    void resize(const Rectangle& setSize){
        clear(); 
        currentSize = setSize;

        float newWidth = currentSize.width / 2.0f, newHeight = currentSize.height / 2.0f;
        float x = currentSize.x, y = currentSize.y;

        childAreas = {
            Rectangle{x + newWidth, y, newWidth, newHeight},
            Rectangle{x, y, newWidth, newHeight},
            Rectangle{x, y + newHeight, newWidth, newHeight},
            Rectangle{x + newWidth, y + newHeight, newWidth, newHeight}
        };

    }

    void clear(){
        particles.clear();

        for(int i = 0; i < 4; i++){
            if(children[i]){
                children[i]->clear();
            }
            children[i].reset();
        }
    }

    void insert(const Particle& newParticle){
        for(int i = 0 ; i < 4; i++){
            if(CheckCollisionPointRec(newParticle.pos, childAreas[i])){
                if(currentDepth + 1 < maxTreeDepth){
                    if(!children[i]){
                        children[i] = std::make_shared<QuadTree>(currentDepth + 1, childAreas[i]);
                    }
                    children[i]->insert(newParticle);
                    return;
                }
            }
        }

        //didn't fit in children, so must go here
        particles.emplace_back(newParticle);
    }

    std::list<Particle> search(Vector2& center, float radius, bool removeSearched){
        std::list<Particle> result;

        // Check if the search area intersects the QuadTree node's boundary
        if(!CheckCollisionCircleRec(center, radius, currentSize)) {
            return result;
        }

        // If this node has particles, add the ones within the search area to the result list
        for(unsigned int i = 0; i < particles.size(); i++){
            if(CheckCollisionPointCircle(particles[i].pos, center, radius)){
                result.push_back(particles[i]);
                if(removeSearched){
                    particles.erase(particles.begin() + i);
                }
            }
        }

        // Recursively search the children nodes
        for(int i = 0; i < 4; i++){
            if(children[i]){
                auto childResult = children[i]->search(center, radius, removeSearched);
                result.splice(result.end(), childResult);
            }
        }

        return result;
    }

    std::vector<Particle> returnAll(int depth){
        std::vector<Particle> result;

        if(currentDepth >= depth){
            result.insert(result.end(), particles.begin(), particles.end());
        }

        for(int i = 0; i < 4; i++){
            if(children[i]){
                auto childResult = children[i]->returnAll(depth);
                result.insert(result.end(), childResult.begin(), childResult.end());
            }
        }

        return result;
    }

    int size() const{
        int count = particles.size();

        for(int i = 0 ; i < 4; i++){
            if(children[i]){
                count += children[i]->size();
            }
        }

        return count;
    }

    void draw() const{
        for(const auto& particle : particles){
            DrawPixelV(particle.pos, particle.color);
        }

        //DrawRectangleLinesEx(currentSize, 0.7, GREEN);

        for(int i = 0; i < 4; i++){
            if(children[i]){
                children[i]->draw();
            }
        }
    }

};

//-------------------------------------------------------------------------------------------------------------------------------------------------------

std::vector<Particle> freeParticles, aggregateParticles;

//-------------------------------------------------------------------------------------------------------------------------------------------------------

std::mt19937 CreateGeneratorWithTimeSeed() {
    // Get the current time in nanoseconds
    auto now = std::chrono::high_resolution_clock::now();
    auto nanos = std::chrono::time_point_cast<std::chrono::nanoseconds>(now).time_since_epoch().count();

    // Create a new mt19937 generator and seed it with the current time in nanoseconds
    std::mt19937 gen(static_cast<unsigned int>(nanos));
    return gen;
}

float RandomFloat(float min, float max, std::mt19937& rng) {
    std::uniform_real_distribution<float> dist(min, max);
    return dist(rng);
}

float vector2distance(Vector2 v1, Vector2 v2) {
    float dx = v2.x - v1.x;
    float dy = v2.y - v1.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<Particle> CreateCircle(int numParticles, Color col, Vector2 center, float radius){
    float degreeIncrement = 360.0f/(float)numParticles;
    std::vector<Particle> particles;

    for(float i = 0; i < 360; i += degreeIncrement){
        float x = radius * cos(i) + center.x;
        float y = radius * sin(i) + center.y;
        Particle p({x, y}, col);
        particles.push_back(p);
    }

    return particles;
}

bool vectorsEqual(Vector2 v1, Vector2 v2){
    if(v1.x == v2.x && v1.y == v2.y){
        return true;
    }
    else{
        return false;
    }
}

void primitiveCollisionCheck(){
    for(unsigned int i = 0; i < aggregateParticles.size(); i++){
        for(unsigned int j = 0; j < freeParticles.size(); j++){
            if(CheckCollisionPointCircle(freeParticles[j].pos, aggregateParticles[i].pos, collisionThreshold)){
                freeParticles[j].color = WHITE;
                aggregateParticles.push_back(freeParticles[j]);
                freeParticles.erase(freeParticles.begin() + j);
            }
        }
    }
}

std::vector<Particle> collisionCheck(QuadTree qt){
    std::list<Particle> result;
    std::vector<Particle> failedCollisions;

    for(auto& aggregateParticle : aggregateParticles){
        result = qt.search(aggregateParticle.pos, collisionThreshold, true);

        for(auto& p : result){
            p.color = WHITE;
            float dist = vector2distance(p.pos, aggregateParticle.pos);

            if(dist >= minimumStickDistance and RandomFloat(0, 1, rng) <= stickingProbability){
                aggregateParticles.push_back(p);
            }
            else{
                p.color = RED;
                p.pos.x = screenWidth;
                failedCollisions.push_back(p);
            }
        }
    }

    return failedCollisions;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------

void Initialize(){
    InitWindow(screenWidth, screenHeight, "DLA, hopefully");
    SetTargetFPS(100);

    constexpr int startingNumParticles = 10000, startingRadius = 100;
    const Color startingColor = RED;
    const Vector2 startingCenter = {screenWidth / 2, screenHeight / 2};

    freeParticles = CreateCircle(startingNumParticles, startingColor, startingCenter, startingRadius);
    aggregateParticles = {1, Particle({screenWidth / 2.0, screenHeight / 2.0}, WHITE)};
}

void DrawParticlesVector(const std::vector<Particle>& particles){
    for(unsigned int i = 0; i < particles.size(); i++){
        DrawPixelV(particles[i].pos, particles[i].color);
    }
}

void RandomWalkAll(std::vector<Particle>& particles){
    for(auto& p : particles){
        p.RandomWalk(2, 1);
    }
}

void ConcentricCircles(int frameCount){
    if(frameCount / 5 < screenHeight / 2 && frameCount % 500 == 0){
        std::vector<Particle> fp2 = CreateCircle(300 * (1 + frameCount / 50),RED,{screenWidth/2.0, screenHeight/2.0}, 50 + frameCount / 5);
        freeParticles.insert(freeParticles.end(), fp2.begin(), fp2.end());
    }
}

QuadTree initializeQT(){
    //Timer t;
    QuadTree qt(0, Rectangle{0, 0, screenWidth, screenHeight});

    for(const auto& p : freeParticles){
        qt.insert(p);
    }

    return qt;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------

int main(){
    Initialize();
    
    for(int frameCount = 0; !WindowShouldClose(); frameCount++){

        //ConcentricCircles(frameCount);
        RandomWalkAll(freeParticles);

        QuadTree qt = initializeQT();
        std::vector<Particle> failedCollisions = collisionCheck(qt);

        freeParticles = qt.returnAll(0);
        for(unsigned int i = 0; i < failedCollisions.size(); i++){
            freeParticles.push_back(failedCollisions[i]);
            freeParticles[freeParticles.size()].RandomWalk(2,1);
        }
        
        BeginDrawing();
        {
            ClearBackground(BLACK);
            DrawFPS(10,10);
            DrawText(TextFormat("%d freeparticles, and %d aggregate particles\t %d total particles", freeParticles.size(), aggregateParticles.size(), freeParticles.size() + aggregateParticles.size()), 10, 30, 10, GREEN);

            DrawParticlesVector(aggregateParticles);

            qt.draw();
        }
        EndDrawing();

        if(maxTreeDepth == 5 and aggregateParticles.size() > 10000){
            maxTreeDepth = 6;
        }

        if(frameCount % 5000 == 0){
            stickingProbability -= 0.01;
            std::cout << stickingProbability << std::endl;
        }

    }

    return 0;
}
