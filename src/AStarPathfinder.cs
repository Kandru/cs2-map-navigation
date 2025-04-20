using CounterStrikeSharp.API;
using CounterStrikeSharp.API.Core;
using CounterStrikeSharp.API.Modules.Utils;
using System.Drawing;

namespace MapNavigation
{
    /// <summary>
    /// A 3D A* pathfinding implementation for CounterStrikeSharp Vectors
    /// </summary>
    public class AStarPathfinder
    {
        // Store a reference to the plugin
        private float _maxStartDistance = 200f;     // Maximum distance allowed from the starting point
        private float _maxGoalDistance = 100f;     // Maximum distance allowed to reach the goal
        private float _addedHeight = 2f;               // Height offset for points returned by the pathfinder
        private float _maxStepDistance = 21f;           // Maximum distance between path nodes
        private float _minStepDistance = 9f;           // Minimum distance between path nodes
        private float _heuristicWeight = 1.0f;          // Weight for the heuristic function (>1 = faster but less optimal paths)
        private int _maxIterations = 100000;             // Safety limit to prevent infinite loops
        // Optimization parameters
        private float _straightLineThreshold = 0.97f;      // Slightly lower to preserve more meaningful points
        private float _cornerRoundingFactor = 0.4f;        // Higher value for more rounded corners
        private int _maxPointsPerCurve = 8;                // More points for smoother curves
        private float _optimizedDistance = 3.0f;        // Smaller minimum distance for more detail
        private float _heightChangeReductionFactor = 0.5f; // Balance between smoothness and height accuracy
        private float _heightChangeThreshold = 0.8f;       // More sensitive to height changes
        // Debugging parameters
        private bool _debug = false;                     // Enable debug mode for pathfinding

        // Configuration accessors
        public float MaxStartDistance
        {
            get => _maxStartDistance;
            set => _maxStartDistance = Math.Max(_maxStepDistance, value);
        }

        public float MaxGoalDistance
        {
            get => _maxGoalDistance;
            set => _maxGoalDistance = Math.Max(_maxStepDistance, value);
        }

        public float AddedHeight
        {
            get => _addedHeight;
            set => _addedHeight = value;
        }

        public float MaxStepDistance
        {
            get => _maxStepDistance;
            set => _maxStepDistance = Math.Max(5f, value);
        }

        public float MinStepDistance
        {
            get => _minStepDistance;
            set => _minStepDistance = Math.Max(1f, Math.Min(value, _maxStepDistance - 5f));
        }

        public float HeuristicWeight
        {
            get => _heuristicWeight;
            set => _heuristicWeight = Math.Max(1f, value);
        }

        public int MaxIterations
        {
            get => _maxIterations;
            set => _maxIterations = Math.Max(1000, value);
        }

        public float StraightLineThreshold
        {
            get => _straightLineThreshold;
            set => _straightLineThreshold = Math.Clamp(value, 0.9f, 0.9999f);
        }

        public float CornerRoundingFactor
        {
            get => _cornerRoundingFactor;
            set => _cornerRoundingFactor = Math.Clamp(value, 0.0f, 1.0f);
        }

        public int MaxPointsPerCurve
        {
            get => _maxPointsPerCurve;
            set => _maxPointsPerCurve = Math.Max(0, value);
        }

        public float OptimizedDistance
        {
            get => _optimizedDistance;
            set => _optimizedDistance = Math.Max(1.0f, value);
        }

        public float HeightChangeReductionFactor
        {
            get => _heightChangeReductionFactor;
            set => _heightChangeReductionFactor = Math.Clamp(value, 0.0f, 1.0f);
        }

        public float HeightChangeThreshold
        {
            get => _heightChangeThreshold;
            set => _heightChangeThreshold = Math.Max(0.0f, value);
        }

        public bool Debug
        {
            get => _debug;
            set => _debug = value;
        }

        /// <summary>
        /// Finds a path between two 3D points using A* pathfinding algorithm.
        /// </summary>
        /// <param name="start">Starting position</param>
        /// <param name="goal">Goal position</param>
        /// <param name="points">List of all available navigation points</param>
        /// <param name="optimizePath">Whether to perform path optimization</param>
        /// <returns>List of Vector points forming the path, or empty list if no path found</returns>
        public List<Vector> FindPath(Vector start, Vector goal, List<Vector> points, bool optimizePath = false)
        {
            // If start and goal are very close, return direct path
            if ((start - goal).Length() < _maxStepDistance)
            {
                return new List<Vector> { start, goal };
            }
            // Add start and goal to points if needed
            bool addedStart = false;
            bool addedGoal = false;

            var workingPoints = new List<Vector>(points);

            // Add start and end to the working set if they're not there
            if (!ContainsVector(workingPoints, start))
            {
                workingPoints.Add(start);
                addedStart = true;
            }
            if (!ContainsVector(workingPoints, goal))
            {
                workingPoints.Add(goal);
                addedGoal = true;
            }
            // Create a lookup for fast distance calculation
            Dictionary<(int, int), float> distanceLookup = new Dictionary<(int, int), float>();

            // Find the path
            var path = FindPathAStar(
                addedStart ? workingPoints.Count - (addedGoal ? 2 : 1) : GetIndexOfVector(workingPoints, start),
                addedGoal ? workingPoints.Count - 1 : GetIndexOfVector(workingPoints, goal),
                workingPoints,
                distanceLookup
            );

            // Convert indices to vectors
            var result = new List<Vector>();
            foreach (int index in path)
            {
                result.Add(workingPoints[index] + new Vector(0, 0, _addedHeight));
            }

            // Optimize the path if requested
            if (optimizePath && result.Count > 2)
            {
                result = OptimizePath(result);
            }

            // Position the last point _maxGoalDistance away from the goal
            if (result.Count >= 2)
            {
                Vector finalPoint = result[^1];
                Vector secondToLastPoint = result[^2];

                // Calculate vector pointing FROM second-to-last point TO goal
                Vector directionToGoal = finalPoint - secondToLastPoint;
                float distance = directionToGoal.Length();

                // Only adjust if the last point is the goal and we have a valid direction
                if (distance > 0.1f && (finalPoint - goal).Length() < 1.0f)
                {
                    // Normalize the direction vector
                    directionToGoal.X /= distance;
                    directionToGoal.Y /= distance;
                    directionToGoal.Z /= distance;

                    // Place a point at a reasonable distance from the goal along the same approach vector
                    // Use a smaller distance (not the full _maxGoalDistance)
                    float approachDistance = Math.Min(_maxStepDistance * 2, _maxGoalDistance);
                    result[result.Count - 1] = goal - (directionToGoal * approachDistance);
                }
            }

            return result;
        }

        /// <summary>
        /// Find path using A* algorithm with indices
        /// </summary>
        private List<int> FindPathAStar(int startIdx, int goalIdx, List<Vector> points, Dictionary<(int, int), float> distanceLookup)
        {
            // A* implementation
            var openSet = new PriorityQueue<PathNode>();
            var closedSet = new HashSet<int>();
            var gScore = new Dictionary<int, float>();
            var cameFrom = new Dictionary<int, int>();

            // For debugging
            var exploredPaths = new Dictionary<int, int>();
            var exploredNodes = new HashSet<int>();

            // Initialize the starting point
            gScore[startIdx] = 0;
            openSet.Enqueue(new PathNode(
                startIdx,
                0,
                CalculateHeuristic(points[startIdx], points[goalIdx]) * _heuristicWeight
            ));

            int iterations = 0;
            while (openSet.Count > 0 && iterations < _maxIterations)
            {
                iterations++;

                // Get the node with the lowest f-score
                var current = openSet.Dequeue();

                // Add to explored nodes for debugging
                exploredNodes.Add(current.Index);
                // Check if we've reached the goal
                if (current.Index == goalIdx)
                {
                    // Path found! Visualize the exploration paths if debug is enabled
                    if (_debug)
                    {
                        VisualizeExploredPaths(points, exploredPaths);
                    }
                    return ReconstructPath(cameFrom, current.Index, startIdx);
                }

                // Mark as processed
                closedSet.Add(current.Index);

                // Process neighbors
                Vector currentPos = points[current.Index];

                for (int i = 0; i < points.Count; i++)
                {
                    // Skip if it's the current node or already processed
                    if (i == current.Index || closedSet.Contains(i))
                        continue;

                    // Calculate or retrieve the distance
                    float distance;
                    var distKey = (Math.Min(current.Index, i), Math.Max(current.Index, i));

                    if (!distanceLookup.TryGetValue(distKey, out distance))
                    {
                        distance = (points[i] - currentPos).Length();
                        distanceLookup[distKey] = distance;
                    }

                    // Special case for the goal - allow reaching it from further away
                    bool isGoalNode = i == goalIdx;
                    bool isStartNode = current.Index == startIdx;
                    float maxAllowedDistance = isGoalNode ? _maxGoalDistance : isStartNode ? _maxStartDistance : _maxStepDistance;
                    float minAllowedDistance = isGoalNode || isStartNode ? 0 : _minStepDistance;

                    // Skip if not a valid connection (too far or too close)
                    if (distance > maxAllowedDistance || distance < minAllowedDistance)
                        continue;

                    // Calculate new g-score
                    float tentativeGScore = gScore[current.Index] + distance;

                    // Check if this is a better path
                    if (!gScore.ContainsKey(i) || tentativeGScore < gScore[i])
                    {
                        // Record this better path
                        cameFrom[i] = current.Index;
                        exploredPaths[i] = current.Index; // For debugging
                        gScore[i] = tentativeGScore;
                        float h = CalculateHeuristic(points[i], points[goalIdx]) * _heuristicWeight;
                        float f = tentativeGScore + h;

                        // Add to open set if not already there
                        if (!openSet.Contains(i))
                        {
                            openSet.Enqueue(new PathNode(i, tentativeGScore, f));
                        }
                        else
                        {
                            // Update priority of existing node
                            openSet.UpdatePriority(i, f);
                        }
                    }
                }
            }

            // No path found, visualize the exploration to see where it got stuck
            if (_debug)
            {
                VisualizeExploredPaths(points, exploredPaths);
                VisualizeExploredNodes(points, exploredNodes, points[startIdx], points[goalIdx]);
            }

            // No path found
            return new List<int>();
        }

        /// <summary>
        /// Visualize all edges the algorithm explored during pathfinding
        /// </summary>
        private void VisualizeExploredPaths(List<Vector> points, Dictionary<int, int> exploredPaths)
        {
            foreach (var kvp in exploredPaths)
            {
                // Use different colors for different segments
                Color color = Color.FromArgb(100, 100, 255); // Light blue
                CreateBeam(points[kvp.Value] + new Vector(0, 0, 10), points[kvp.Key] + new Vector(0, 0, 10), color, 0.5f, 5.0f);
            }
        }

        /// <summary>
        /// Visualize all nodes the algorithm explored, the start, and the goal
        /// </summary>
        private void VisualizeExploredNodes(List<Vector> points, HashSet<int> exploredNodes, Vector start, Vector goal)
        {
            // Draw start point in green
            CreateSphere(start, Color.Green, 8.0f, 5.0f);

            // Draw goal point in red
            CreateSphere(goal, Color.Red, 8.0f, 5.0f);

            // Draw all explored nodes in yellow
            foreach (int nodeIdx in exploredNodes)
            {
                // Skip start and goal as they get special colors
                if ((points[nodeIdx] - start).Length() < 1.0f || (points[nodeIdx] - goal).Length() < 1.0f)
                    continue;

                CreateSphere(points[nodeIdx], Color.Yellow, 3.0f, 5.0f);
            }
        }

        /// <summary>
        /// Create a sphere visualization at a point (using beams to approximate)
        /// </summary>
        private void CreateSphere(Vector position, Color color, float size = 5.0f, float duration = 2.0f)
        {
            // Create a sphere-like visual using beams
            float halfSize = size / 2;

            // Create cross beams to represent the sphere
            CreateBeam(
                new Vector(position.X - halfSize, position.Y, position.Z),
                new Vector(position.X + halfSize, position.Y, position.Z),
                color, halfSize / 5, duration);

            CreateBeam(
                new Vector(position.X, position.Y - halfSize, position.Z),
                new Vector(position.X, position.Y + halfSize, position.Z),
                color, halfSize / 5, duration);

            CreateBeam(
                new Vector(position.X, position.Y, position.Z - halfSize),
                new Vector(position.X, position.Y, position.Z + halfSize),
                color, halfSize / 5, duration);
        }

        private void CreateBeam(Vector startOrigin, Vector endOrigin, Color? color = null, float width = 1f, float timeout = 2f)
        {
            Server.NextFrame(() =>
            {
                color ??= Color.White;
                CEnvBeam? beam = Utilities.CreateEntityByName<CEnvBeam>("env_beam");
                if (beam == null) return;
                beam.Width = width;
                beam.Render = color.Value;
                beam.SetModel("materials/sprites/laserbeam.vtex");
                beam.Teleport(startOrigin);
                beam.EndPos.X = endOrigin.X;
                beam.EndPos.Y = endOrigin.Y;
                beam.EndPos.Z = endOrigin.Z;
                Utilities.SetStateChanged(beam, "CBeam", "m_vecEndPos");
            });
        }

        /// <summary>
        /// Calculates the heuristic value (estimated distance) between two points
        /// </summary>
        private float CalculateHeuristic(Vector a, Vector b)
        {
            return (a - b).Length();
        }

        /// <summary>
        /// Reconstructs the path from the goal back to the start
        /// </summary>
        private List<int> ReconstructPath(Dictionary<int, int> cameFrom, int current, int start)
        {
            var path = new List<int> { current };

            while (current != start)
            {
                current = cameFrom[current];
                path.Insert(0, current);
            }

            return path;
        }

        /// <summary>
        /// Optimizes the path by removing redundant points and smoothing corners
        /// </summary>
        /// <param name="path">The original path to optimize</param>
        private List<Vector> OptimizePath(List<Vector> path)
        {
            if (path.Count <= 2)
                return path;

            // First pass: remove points that are nearly collinear
            // BUT preserve points before and after significant height changes
            var simplifiedPath = new List<Vector> { path[0] };
            for (int i = 1; i < path.Count - 1; i++)
            {
                // Check if this point or the next point has a significant height change
                bool hasHeightChange = false;
                if (i > 0 && Math.Abs(path[i].Z - path[i - 1].Z) > _heightChangeThreshold)
                    hasHeightChange = true;
                if (i < path.Count - 1 && Math.Abs(path[i + 1].Z - path[i].Z) > _heightChangeThreshold)
                    hasHeightChange = true;

                // Keep the point if it's not collinear OR if there's a height change
                if (hasHeightChange || !IsApproximatelyLinear(
                    simplifiedPath[simplifiedPath.Count - 1],
                    path[i],
                    path[i + 1],
                    _straightLineThreshold))
                {
                    simplifiedPath.Add(path[i]);
                }
            }
            simplifiedPath.Add(path[path.Count - 1]);

            // If after simplification we have 2 or fewer points, just return them
            if (simplifiedPath.Count <= 2)
                return simplifiedPath;

            // Second pass: Add intermediate points to smooth corners
            var smoothedPath = new List<Vector> { simplifiedPath[0] };

            for (int i = 1; i < simplifiedPath.Count - 1; i++)
            {
                Vector prev = simplifiedPath[i - 1];
                Vector current = simplifiedPath[i];
                Vector next = simplifiedPath[i + 1];

                // Calculate vectors and angles
                Vector prevToCurrent = current - prev;
                Vector currentToNext = next - current;
                float fromPrevLength = prevToCurrent.Length();
                float toNextLength = currentToNext.Length();

                // Skip very short segments
                if (fromPrevLength < 1.0f || toNextLength < 1.0f) continue;

                // Normalize vectors for direction calculations
                Vector fromPrevNorm = prevToCurrent;
                Vector toNextNorm = currentToNext;
                NormalizeInPlace(ref fromPrevNorm);
                NormalizeInPlace(ref toNextNorm);

                // Calculate dot product to determine angle
                float dotProduct = fromPrevNorm.X * toNextNorm.X +
                          fromPrevNorm.Y * toNextNorm.Y +
                          fromPrevNorm.Z * toNextNorm.Z;

                // Check for significant height changes
                float heightChangePrev = Math.Abs(prev.Z - current.Z);
                float heightChangeNext = Math.Abs(next.Z - current.Z);
                bool hasSignificantHeightChange =
                    heightChangePrev > _heightChangeThreshold ||
                    heightChangeNext > _heightChangeThreshold;

                // Adjust rounding factor based on conditions
                float actualRoundingFactor = _cornerRoundingFactor;
                if (hasSignificantHeightChange)
                {
                    actualRoundingFactor *= _heightChangeReductionFactor;
                }

                // Calculate how many points to add based on angle and distances
                int pointsToAdd = 0;
                if (dotProduct < _cornerRoundingFactor) // Sharper turns get more points
                {
                    float angleBasedFactor = (1.0f - dotProduct) * 10;
                    float distanceFactor = Math.Min(fromPrevLength, toNextLength) / _maxStepDistance;
                    pointsToAdd = Math.Min(_maxPointsPerCurve, (int)(angleBasedFactor * distanceFactor));

                    // Ensure we don't add too many points for very short segments
                    pointsToAdd = Math.Min(pointsToAdd, (int)(Math.Min(fromPrevLength, toNextLength) / 3.0f));
                }

                if (pointsToAdd > 0)
                {
                    // Add intermediate points to smooth the corner
                    for (int p = 1; p <= pointsToAdd; p++)
                    {
                        float t = p / (float)(pointsToAdd + 1);

                        // Improved interpolation that won't cause backtracking
                        // Direct linear interpolation between points
                        Vector prevPoint = prev + (prevToCurrent * t);
                        Vector nextPoint = next - (currentToNext * (1 - t));

                        // Weight the blending more toward the first path at the start, more toward the second path at the end
                        float blendWeight = t;
                        Vector blendedPoint = new Vector(
                            (1 - blendWeight) * prevPoint.X + blendWeight * nextPoint.X,
                            (1 - blendWeight) * prevPoint.Y + blendWeight * nextPoint.Y,
                            (1 - blendWeight) * prevPoint.Z + blendWeight * nextPoint.Z
                        );

                        // Apply curve factor - stronger in the middle of the curve, weaker at endpoints
                        // This pulls the point toward the corner point
                        float curveFactor = actualRoundingFactor * (1 - Math.Abs(2 * t - 1));

                        Vector smoothedPoint = new Vector(
                            (1 - curveFactor) * blendedPoint.X + curveFactor * current.X,
                            (1 - curveFactor) * blendedPoint.Y + curveFactor * current.Y,
                            (1 - curveFactor) * blendedPoint.Z + curveFactor * current.Z
                        );

                        smoothedPath.Add(smoothedPoint);
                    }
                }

                smoothedPath.Add(current);
            }

            // Add the final point
            smoothedPath.Add(simplifiedPath[simplifiedPath.Count - 1]);

            return smoothedPath;
        }

        /// <summary>
        /// Determines if three points are approximately in a straight line
        /// </summary>
        private bool IsApproximatelyLinear(Vector a, Vector b, Vector c, float threshold = 0.95f)
        {
            if ((b - a).Length() < 0.1f || (c - b).Length() < 0.1f)
                return true; // Points are too close

            // Normalize direction vectors
            Vector dir1 = b - a;
            Vector dir2 = c - b;
            NormalizeInPlace(ref dir1);
            NormalizeInPlace(ref dir2);

            // Calculate dot product (cosine of angle between vectors)
            float dotProduct = dir1.X * dir2.X + dir1.Y * dir2.Y + dir1.Z * dir2.Z;

            // If dot product is close to 1, the directions are nearly the same (linear)
            return dotProduct > threshold;
        }

        /// <summary>
        /// Creates points to round a corner
        /// </summary>
        private (Vector?, Vector?) RoundCorner(Vector prev, Vector corner, Vector next, float factor)
        {
            // Calculate vectors
            Vector toPrev = prev - corner;
            Vector toNext = next - corner;

            // Calculate distances
            float distToPrev = toPrev.Length();
            float distToNext = toNext.Length();

            // Skip rounding if points are too close
            if (distToPrev < 0.5f || distToNext < 0.5f)
                return (null, null);

            // Limit rounding to not exceed half the distance to each point
            float maxDistPrev = Math.Min(distToPrev * 0.5f, _maxStepDistance * 0.5f);
            float maxDistNext = Math.Min(distToNext * 0.5f, _maxStepDistance * 0.5f);

            // Normalize vectors
            NormalizeInPlace(ref toPrev);
            NormalizeInPlace(ref toNext);

            // Calculate rounding points
            Vector? before = corner + toPrev * Math.Min(distToPrev * factor, maxDistPrev);
            Vector? after = corner + toNext * Math.Min(distToNext * factor, maxDistNext);

            return (before, after);
        }

        /// <summary>
        /// Normalizes a vector in place
        /// </summary>
        private static void NormalizeInPlace(ref Vector vector)
        {
            float length = vector.Length();
            if (length > 0.001f)
            {
                vector.X /= length;
                vector.Y /= length;
                vector.Z /= length;
            }
        }

        /// <summary>
        /// Check if a list of vectors contains a specific vector
        /// </summary>
        private bool ContainsVector(List<Vector> vectors, Vector vector)
        {
            const float epsilon = 0.1f;

            foreach (var v in vectors)
            {
                if (Math.Abs(v.X - vector.X) < epsilon &&
                    Math.Abs(v.Y - vector.Y) < epsilon &&
                    Math.Abs(v.Z - vector.Z) < epsilon)
                {
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// Get the index of a vector in a list
        /// </summary>
        private int GetIndexOfVector(List<Vector> vectors, Vector vector)
        {
            const float epsilon = 0.1f;

            for (int i = 0; i < vectors.Count; i++)
            {
                if (Math.Abs(vectors[i].X - vector.X) < epsilon &&
                    Math.Abs(vectors[i].Y - vector.Y) < epsilon &&
                    Math.Abs(vectors[i].Z - vector.Z) < epsilon)
                {
                    return i;
                }
            }

            return -1;
        }

        #region Helper Classes for A* Implementation

        /// <summary>
        /// Represents a node in the A* path
        /// </summary>
        private class PathNode : IComparable<PathNode>
        {
            public int Index { get; }
            public float GScore { get; } // Cost from start
            public float FScore { get; } // Total cost (g + h)

            public PathNode(int index, float gScore, float fScore)
            {
                Index = index;
                GScore = gScore;
                FScore = fScore;
            }

            public int CompareTo(PathNode? other)
            {
                if (other == null) return 1;
                return FScore.CompareTo(other.FScore);
            }
        }

        /// <summary>
        /// Priority queue for A* search
        /// </summary>
        private class PriorityQueue<T> where T : PathNode
        {
            private List<T> _heap = new List<T>();
            private Dictionary<int, int> _indices = new Dictionary<int, int>();

            public int Count => _heap.Count;

            public void Enqueue(T item)
            {
                // Add to end of heap
                _heap.Add(item);
                int index = _heap.Count - 1;
                _indices[item.Index] = index;

                // Bubble up
                BubbleUp(index);
            }

            public T Dequeue()
            {
                if (_heap.Count == 0)
                    throw new InvalidOperationException("Queue is empty");

                T result = _heap[0];
                _indices.Remove(result.Index);

                // Move last item to top and bubble down
                int lastIndex = _heap.Count - 1;
                if (lastIndex > 0)
                {
                    _heap[0] = _heap[lastIndex];
                    _indices[_heap[0].Index] = 0;
                    _heap.RemoveAt(lastIndex);
                    BubbleDown(0);
                }
                else
                {
                    _heap.RemoveAt(0);
                }

                return result;
            }

            public bool Contains(int index)
            {
                return _indices.ContainsKey(index);
            }

            public void UpdatePriority(int index, float newFScore)
            {
                if (!_indices.TryGetValue(index, out int heapIndex))
                    return;

                // Create new node with updated score
                T node = _heap[heapIndex];
                T newNode = (T)Activator.CreateInstance(typeof(T), index, node.GScore, newFScore)!;

                // Replace node and fix heap
                _heap[heapIndex] = newNode;

                // Bubble up or down as needed
                if (newFScore < node.FScore)
                    BubbleUp(heapIndex);
                else
                    BubbleDown(heapIndex);
            }

            private void BubbleUp(int index)
            {
                while (index > 0)
                {
                    int parentIndex = (index - 1) / 2;
                    if (_heap[parentIndex].FScore <= _heap[index].FScore)
                        break;

                    // Swap with parent
                    SwapNodes(index, parentIndex);

                    // Move up
                    index = parentIndex;
                }
            }

            private void BubbleDown(int index)
            {
                int lastIndex = _heap.Count - 1;
                while (true)
                {
                    int leftChild = 2 * index + 1;
                    int rightChild = 2 * index + 2;
                    int smallest = index;

                    // Find smallest of parent, left child, right child
                    if (leftChild <= lastIndex && _heap[leftChild].FScore < _heap[smallest].FScore)
                        smallest = leftChild;

                    if (rightChild <= lastIndex && _heap[rightChild].FScore < _heap[smallest].FScore)
                        smallest = rightChild;

                    if (smallest == index)
                        break;

                    // Swap with smallest child
                    SwapNodes(index, smallest);

                    // Move down
                    index = smallest;
                }
            }

            private void SwapNodes(int i, int j)
            {
                T temp = _heap[i];
                _heap[i] = _heap[j];
                _heap[j] = temp;

                // Update indices
                _indices[_heap[i].Index] = i;
                _indices[_heap[j].Index] = j;
            }
        }

        #endregion
    }
}