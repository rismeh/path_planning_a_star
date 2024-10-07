"""Main window for path planning application.

Author: Marcel Schindhelm, Marian Friedrich

Copyright: German copyright law 2021

Date created: 01.12.2021

Date last modified: 14.12.2021

Python Version: 3.9.7
"""

# System imports
import sys
import traceback
from copy import deepcopy
from math import atan2, cos, pi, sin

# User interface imports
from PyQt5 import QtCore
from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QBrush, QColor, QFont, QPainter, QPixmap
from PyQt5.QtWidgets import QAction, QApplication, QFileDialog, QGraphicsEllipseItem, QGraphicsScene,\
    QGraphicsTextItem, QGraphicsView, QMainWindow

# Algorithm implementations
from a_star import AStarAlgo

from dijkstra import DijkstraAlgo

from floyd_warshall import FloydWarshallAlgo

# Map data structures and parsing
from map_graph import MapGraph

from map_parser import MapParser

# Performance measurement of algorithms
from performance_measurement import PerformanceMeasurement


class NodeVisualization(QGraphicsEllipseItem):
    """UI Element for visualizing nodes."""

    # Variables for selecting the target path
    selectedStartNodeId = -1
    selectedEndNodeId = -1

    def __init__(self, node, radius, pathSelectedCallback):
        """Initialize the Node UI Element.

        Keyword arguments:
        node -- the node of the map structure
        radius -- the radius of the rendered circle
        pathSelectedCallback -- mouse press callback for selecting the node
        """
        QGraphicsEllipseItem.__init__(
            self, node.xPos - radius,
            node.yPos - radius,
            radius * 2, radius * 2)

        self.node = node
        self.setBrush(QBrush(QColor(255, 255, 0, 255)))  # Fill color
        self.setPen(QColor(0, 0, 0, 255))  # Border color
        self.pathSelectedCallback = pathSelectedCallback

    def mousePressEvent(self, event):
        """Select target path on mouse press.

        Keyword arguments:
        event -- the mouse press event from qt
        """
        if event.button() != Qt.LeftButton:
            return
        if NodeVisualization.selectedStartNodeId == -1:
            NodeVisualization.selectedStartNodeId = self.node.id
            self.setBrush(QBrush(QColor(255, 0, 0, 255)))    # Fill color
            return
        elif NodeVisualization.selectedEndNodeId == -1:
            NodeVisualization.selectedEndNodeId = self.node.id
            self.pathSelectedCallback(
                NodeVisualization.selectedStartNodeId,
                NodeVisualization.selectedEndNodeId)
            return
        else:
            NodeVisualization.selectedStartNodeId = -1
            NodeVisualization.selectedEndNodeId = -1
            self.mousePressEvent(event)


class NodeTagVisualization(QGraphicsTextItem):
    """UI Element for map node IDs."""

    def __init__(self, node):
        """Initialize values of node tag ids.

        Keyword arguments:
        node -- the node from the map structure
        """
        QGraphicsEllipseItem.__init__(self, str(node.id))

        self.setPos(node.xPos - self.boundingRect().width() / 2,
                    node.yPos - self.boundingRect().height() / 2)
        self.setDefaultTextColor(QColor(0, 0, 0, 255))


class GraphicsScene(QGraphicsScene):
    """Main graphic scene of the path planner."""

    def __init__(self, pathSelectedCallback, parent=None):
        """Initialize the graphics scene for the pathplanner."""
        QGraphicsScene.__init__(self, parent)
        self.setSceneRect(-100, -650, 1200, 800)
        self.opt = ''

        self.nodeFillColor = QBrush(QColor(255, 255, 0, 255))
        self.nodeBorderColor = QColor(0, 0, 0, 255)
        self.edgeColor = QColor(0, 0, 0, 255)

        self.radius = 15
        self.annotationDistance = 20
        self.mapGraph = None
        self.arrowTipLenght = 10

        self.pathSelectedCallback = pathSelectedCallback

        textHint = self.addText('Select File >> Load (to open a Map file (.txt))')
        textHint.setFont(QFont('Calibri', pointSize=10))
        textHint.setScale(1.5)

        x = self.width() / 2 - textHint.boundingRect().width() + self.sceneRect().x()
        y = self.height() / 2 - textHint.boundingRect().height() / 2 + self.sceneRect().y()
        textHint.setPos(x, y)

    def setMap(self, mapGraph: MapGraph):
        """Change the currently shown map.

        Keyword arguments:
        mapGraph -- the new map data structure
        """
        self.mapGraph = deepcopy(mapGraph)
        self.convertMapCoordinates()
        self.renderMap()

    def setOption(self, opt):
        """Set the option of the graphics scene."""
        self.opt = opt

    def renderMap(self):
        """Draw the map to the graphics scene."""
        if self.mapGraph is None:
            return

        self.clear()

        for key in self.mapGraph.nodePositionDict.keys():
            node = self.mapGraph.nodePositionDict[key]
            self.drawMapNode(node, QBrush(QColor(255, 255, 0, 255)))

        for key in self.mapGraph.edgeDict.keys():
            startNode = self.mapGraph.nodePositionDict[key]
            edges = self.mapGraph.edgeDict[key]
            for edge in edges:
                endNode = self.mapGraph.nodePositionDict[edge.endNodeId]
                self.drawMapEdge(startNode, endNode, edge.cost, self.edgeColor)

    def highLightPath(self, path, infoText=''):
        """Highlight the shortest path between nodes.

        Keyword arguments:
        path -- list of node ids of the shortest path
        infoText -- additional text (shown in the graphic scene)
        """
        self.renderMap()

        text = self.addText(str(infoText))
        text.setDefaultTextColor(QColor(0, 0, 0, 255))
        text.setScale(1.15)
        text.setPos(-10, -10)

        # In case no path was found, return
        if path is None:
            return

        previousNode = None
        i = 0
        for nodeID in path:
            node = self.mapGraph.nodePositionDict[nodeID]
            if i == 0:
                self.drawMapNode(node, QBrush(QColor(0, 0, 255, 255)))
            elif i == len(path) - 1:
                self.drawMapNode(node, QBrush(QColor(0, 255, 255, 255)))
            else:
                self.drawMapNode(node, QBrush(QColor(0, 126, 125, 255)))
            if previousNode is not None:
                color = QColor(0, 126, 125, 255)
                self.drawMapEdge(previousNode, node, 0, color, drawText=False)
            previousNode = node
            i += 1

    def convertMapCoordinates(self):
        """Convert map coordinates to start from bottom left corner."""
        for key in self.mapGraph.nodePositionDict.keys():
            node = self.mapGraph.nodePositionDict[key]
            node.yPos = -node.yPos

    def drawMapNode(self, node, color: QColor):
        """Draw the node of a map as a circle.

        Keyword arguments:
        node -- the node from the map data
        color -- the background color of the node
        """
        nodeVisual = NodeVisualization(
            node, self.radius, self.pathSelectedCallback)
        nodeVisual.setBrush(color)
        self.addItem(nodeVisual)
        nodeTag = NodeTagVisualization(node)
        self.addItem(nodeTag)

    def drawMapEdge(self, startNode, endNode, cost, color: QColor, drawText=True):
        """Draw the edge of a map.

        Keyword arguments:
        startNode -- the first node from the map data
        endNode -- the second node from the map data
        cost -- the costs of the edge
        color -- the color of the arrow to be drawn
        drawText -- if true, draw the cost value
        """
        angle = atan2(
            endNode.yPos - startNode.yPos,
            endNode.xPos - startNode.xPos)

        # Calculate start and end point of arrow
        startPoint = QPointF(
            startNode.xPos + cos(angle) * self.radius,
            startNode.yPos + sin(angle) * self.radius)
        endPoint = QPointF(
            endNode.xPos - cos(angle) * self.radius,
            endNode.yPos - sin(angle) * self.radius)

        self.addLine(startPoint.x(), startPoint.y(), endPoint.x(), endPoint.y(), color)

        rotatedVector = QPointF(
            cos(angle - pi / 2) * self.annotationDistance,
            sin(angle - pi / 2) * self.annotationDistance)

        weightedCenterPos = QPointF(
            (endNode.xPos * 2 + startNode.xPos) / 3,
            (endNode.yPos * 2 + startNode.yPos) / 3)

        bottomLeftTextPos = QPointF(
            weightedCenterPos.x() + rotatedVector.x(),
            weightedCenterPos.y() + rotatedVector.y())

        # Add arrow tips
        arrowRotatedVector = QPointF(
            cos(angle - 3 * pi / 4) * self.arrowTipLenght,
            sin(angle - 3 * pi / 4) * self.arrowTipLenght)
        arrowEndPos = QPointF(
            endPoint.x() + arrowRotatedVector.x(),
            endPoint.y() + arrowRotatedVector.y())
        self.addLine(endPoint.x(), endPoint.y(), arrowEndPos.x(), arrowEndPos.y(), color)

        arrowRotatedVector = QPointF(
            cos(angle + 3 * pi / 4) * self.arrowTipLenght,
            sin(angle + 3 * pi / 4) * self.arrowTipLenght)
        arrowEndPos = QPointF(
            endPoint.x() + arrowRotatedVector.x(),
            endPoint.y() + arrowRotatedVector.y())
        self.addLine(endPoint.x(), endPoint.y(), arrowEndPos.x(), arrowEndPos.y(), color)

        if drawText:
            text = self.addText(str(cost))
            text.setDefaultTextColor(color)
            text.setPos(bottomLeftTextPos.x() - text.boundingRect().width() / 2,
                        bottomLeftTextPos.y() - text.boundingRect().height() / 2)

    def pathSelectedCallback(self, startId, endNodeId):
        """Call the registered callback function.

        Keyword arguments:
        startId -- the start node id of the event callback
        endNodeId --  the end node id of the event callback
        """
        self.pathSelectedCallback(startId, endNodeId)

    def mousePressEvent(self, event):
        """Re-render the map if path selected is reset.

        Keyword arguments:
        event -- the mouse press event from qt
        """
        pathSelected = NodeVisualization.selectedStartNodeId != -1 \
            and NodeVisualization.selectedEndNodeId != -1
        if pathSelected:
            self.renderMap()
        super(GraphicsScene, self).mousePressEvent(event)


class GraphicsView(QGraphicsView):
    """Graphic view of the path planning window."""

    def __init__(self, scene, parent=None):
        """Initialize the graphics view.

        Keyword arguments:
        scene -- the target grahic scene from qt
        """
        QGraphicsView.__init__(self, scene, parent)

        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.startPos = None
        self.scaleFactor = 1.1
        self.currentScale = 1.0
        self.maxScaleFactor = 1.25
        self.minScaleFactor = 0.4

    def wheelEvent(self, event):
        """Zoom if the mouse wheel changes.

        Keyword arguments:
        event -- the mouse wheel event from qt
        """
        if event.angleDelta().y() > 0:
            scale = self.scaleFactor * self.currentScale
            if (scale < self.maxScaleFactor):
                self.currentScale = scale
                self.scale(self.scaleFactor, self.scaleFactor)  # Zoom in
        else:
            scale = (1.0/self.scaleFactor) * self.currentScale
            if (scale > self.minScaleFactor):
                self.currentScale = scale
                self.scale(1.0 / self.scaleFactor, 1.0 / self.scaleFactor)  # Zooming out
        super(GraphicsView, self).wheelEvent(event)

    def mousePressEvent(self, event):
        """Grab the position of the mouse.

        Keyword arguments:
        event -- the mouse press event from qt
        """
        grabKey = event.modifiers() & Qt.ControlModifier \
            and event.button() == Qt.LeftButton \
            or event.button() == Qt.MiddleButton
        if grabKey:
            self.startPos = event.pos()  # store the origin point
        else:
            super(GraphicsView, self).mousePressEvent(event)

    def mouseMoveEvent(self, event):
        """Move view when mouse is dragged.

        Keyword arguments:
        event -- the mouse move event from qt
        """
        if self.startPos is not None:
            delta = self.startPos - event.pos()
            transform = self.transform()
            deltaX = delta.x() / transform.m11()
            deltaY = delta.y() / transform.m22()
            self.setSceneRect(self.sceneRect().translated(deltaX, deltaY))
            self.startPos = event.pos()
        else:
            super(GraphicsView, self).mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        """Reset start pos when the mouse is released.

        Keyword arguments:
        event -- the mouse release event from qt
        """
        self.startPos = None
        super(GraphicsView, self).mouseReleaseEvent(event)


class Window(QMainWindow):
    """Main window of the pathplanner application."""

    def __init__(self):
        """Initialize the main window."""
        super().__init__()

        self.mapGraph = None
        self.title = 'Pathplanner'
        self.setWindowTitle(self.title)
        self.setGeometry(100, 100, 1200, 800)

        # Create grahic view/scene
        self.scene = GraphicsScene(self.pathSelectedCallback)
        self.graphicView = GraphicsView(self.scene, self)

        self.selectedAlgorithm = None
        self.selectableAlgorithms = ['A Star', 'Dijkstra', 'Floyd-Warshall']

        # Algorithm solver classes
        self.floydWarshall = FloydWarshallAlgo()
        self.dijkstra = DijkstraAlgo()
        self.astar = AStarAlgo()

        # creating menu bar
        self.createMainMenu()
        self.show()

    def createMainMenu(self):
        """Generate the main menu of the window."""
        mainMenu = self.menuBar()
        fileMenu = mainMenu.addMenu('File')

        # Create action for loading maps
        loadAction = QAction('Load', self)
        loadAction.setShortcut('Ctrl + L')
        fileMenu.addAction(loadAction)
        loadAction.triggered.connect(self.openFileNameDialog)

        # Create action for exporting images
        saveAction = QAction('Export Image', self)
        saveAction.setShortcut('Ctrl + S')
        fileMenu.addAction(saveAction)
        saveAction.triggered.connect(self.exportAsImage)

        # Create menu with a checkable list for algorithms
        self.algorithmsMenu = mainMenu.addMenu('Algorithms')
        self.algorithmsMenu.triggered.connect(self.changeAlgorithm)

        for i in range(0, len(self.selectableAlgorithms)):
            algoAction = self.algorithmsMenu.addAction(
                self.selectableAlgorithms[i])
            algoAction.setCheckable(True)
            if i == 0:
                algoAction.setChecked(True)
                self.selectedAlgorithm = self.selectableAlgorithms[i]

    def changeAlgorithm(self, action):
        """Change the selected algorithm."""
        for a in self.algorithmsMenu.actions():
            if a.text() != action.text():
                a.setChecked(False)
        self.selectedAlgorithm = action.text()

    def exportAsImage(self):
        """Export graphic view as image (.png)."""
        pixmap = QPixmap(self.graphicView.viewport().size())
        self.graphicView.viewport().render(pixmap)

        name = QFileDialog.getSaveFileName(self, 'Save File', '', '(*.png)')
        pixmap.save(name[0])

    def openFileNameDialog(self):
        """Get location of a map file and open the map."""
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getOpenFileName(
            self,
            'Open map file', '',
            'Map Files (*.txt)',
            options=options)
        if fileName:
            mapParser = MapParser()
            try:
                self.mapGraph = mapParser.ParseMapData(fileName)
            except AssertionError:
                _, _, tb = sys.exc_info()
                traceback.print_tb(tb)  # Fixed format
                tb_info = traceback.extract_tb(tb)
                filename, line, func, text = tb_info[-1]
                print(f'Error occurred on line {line} in map parser: {text}')
                return
            self.scene.setMap(self.mapGraph)

    def resizeEvent(self, event):
        """Resize graphic view.

        Keyword arguments:
        event -- the window resize event from qt
        """
        self.graphicView.setGeometry(
            0, 0,
            self.frameGeometry().width(),
            self.frameGeometry().height())

    def keyPressEvent(self, event):
        """React to keyboard input.

        Keyword arguments:
        event -- the key press event from qt
        """
        # Switch between fullscreen and windowed mode
        if event.key() == QtCore.Qt.Key_F11 or event.key() == QtCore.Qt.Key_F:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
        # Re-render the map on escape key
        if event.key() == QtCore.Qt.Key_Escape:
            self.scene.renderMap()

    def pathSelectedCallback(self, startId, endId):
        """Run the path planning when both nodes are selected.

        It finds the shortest path between startId and endID with
        the selected algorithm.

        Keyword arguments:
        startId -- the start node identifier
        endId -- the end node identifier
        """
        if self.selectedAlgorithm == 'A Star':
            # Measure time as well
            path, elapsedTime = PerformanceMeasurement.measureExecutionTime(
                self.astar.solveAlgoritm, startId, endId, self.mapGraph)

            if path is None:
                infoText = 'No path found. (A* took %d micro seconds)'
            else:
                infoText = 'A* took %d micro seconds to find a path.'

            self.scene.highLightPath(path, infoText % (elapsedTime))
        elif self.selectedAlgorithm == 'Dijkstra':
            # Measure time as well
            path, elapsedTime = PerformanceMeasurement.measureExecutionTime(
                self.dijkstra.solveAlgoritm, startId, endId, self.mapGraph)

            if path is None:
                infoText = 'No path found. (Dijkstra took %d micro seconds)'
            else:
                infoText = 'Dijkstra took %d micro seconds to find a path.'

            self.scene.highLightPath(path, infoText % (elapsedTime))
        elif self.selectedAlgorithm == 'Floyd-Warshall':
            # Measure time as well
            path, elapsedTime = PerformanceMeasurement.measureExecutionTime(
                self.floydWarshall.solveAlgoritm,
                startId, endId,
                self.mapGraph)

            if path is None:
                infoText = 'No path found. (Floyd Warshall took %d micro seconds)'
            else:
                infoText = 'Floyd Warshall took %d micro seconds to find a path.'

            self.scene.highLightPath(path, infoText % (elapsedTime))
        else:
            # TODO: ADD OTHER ALGORITHMS HERE
            infoText = 'Algorithm not implemented'
            print(infoText)
            self.scene.highLightPath([], infoText)


def main():
    """Start the user interface application."""
    App = QApplication(sys.argv)
    window = Window()
    window  # prevent unused variable
    sys.exit(App.exec())


if __name__ == '__main__':
    main()
