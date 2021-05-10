using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Pathfinding : MonoBehaviour
{
    public ArrowPointer arrow;
    private const int MOVE_STRAIGHT_COST = 10;
    private const int MOVE_DIAGONAL_COST = 14;

    Grid grid;
    private Heap<Node> openSet;
    private List<Node> closedSet;
    private void Awake()
    {
        grid = GetComponent<Grid>();
    }
    public void FindPath(Vector3 startPos, Vector3 targetPos)
    {
        Node startNode = grid.NodeFromWorldPoint(startPos);
        Node targetNode = grid.NodeFromWorldPoint(targetPos);
        Vector3[] waypoints = new Vector3[0];
        bool pathSuccess = false;
        arrow.follow = pathSuccess;

        openSet = new Heap<Node>(grid.MaxSize);
        closedSet = new List<Node>();
        openSet.Add(startNode);

        while (openSet.Count > 0)
        {
            Node currentNode = openSet.RemoveFirst();
            if (currentNode == targetNode)
            {
                pathSuccess = true;
                break;
            }

            closedSet.Add(currentNode);

            foreach (Node neighbour in grid.GetNeighbours(currentNode))
            {
                if (!neighbour.walkable || closedSet.Contains(neighbour))
                {
                    continue;
                }
                int newMovementCostToNeighbour = currentNode.gCost + CalculateDistance(currentNode, neighbour);
                if (newMovementCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
                {
                    neighbour.gCost = newMovementCostToNeighbour;
                    neighbour.hCost = CalculateDistance(neighbour, targetNode);
                    neighbour.parent = currentNode;
                    if (!openSet.Contains(neighbour))
                        openSet.Add(neighbour);
                }
            }
        }
        if (pathSuccess)
        {
            waypoints = RetracePath(startNode, targetNode);
            arrow.waypoints = null;
            arrow.follow = pathSuccess;
            arrow.waypoints = waypoints;
        }
    }

    Vector3[] RetracePath(Node startNode, Node endNode)
    {
        List<Node> path = new List<Node>();
        Node currentNode = endNode;

        while (currentNode != startNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.parent;
        }

       
        Vector3[] waypoints = simplifyPath(path);
        Array.Reverse(waypoints);

        grid.path = path;
        return waypoints;
    }

    Vector3[] simplifyPath(List<Node> path)
    {
        List<Vector3> waypoints = new List<Vector3>();
        Vector2 dirOld = Vector2.zero;
        for (int i = 1; i < path.Count; i++)
        {
            Vector2 dirNew = new Vector2(path[i - 1].gridX - path[i].gridX, path[i - 1].gridY - path[i].gridY);
            if (dirNew != dirOld)
            {
                waypoints.Add(path[i].worldPosition);
            }
            dirOld = dirNew;
        }
        return waypoints.ToArray();
    }

    int CalculateDistance(Node nodeA, Node nodeB)
    {
        // octile distance
        int xDistance = Mathf.Abs(nodeA.gridX - nodeB.gridX);
        int yDistance = Mathf.Abs(nodeA.gridY - nodeB.gridY);
        int remaining = Mathf.Abs(xDistance - yDistance);
        return MOVE_DIAGONAL_COST * Mathf.Min(xDistance, yDistance) + MOVE_STRAIGHT_COST * remaining;
    }
}
