// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperShuffle extends SubsystemBase {

  Arm subArm;

  int gridSize = 2;
  int gridRow = 0;

  int hybridRow = 0;
  int midRow = 1;
  int highRow = 2;

  int gridOneColumn = 0;
  int gridTwoColumn = gridOneColumn + gridSize;
  int gridThreeColumn = gridOneColumn + gridSize * 2;

  int cellSize = 1;

  String coneColor = "#fffb00";
  String cubeColor = "#a300c4";
  String hybridColor = "#00ff00";
  String gridColor = "#ffffff";
  String offColor = "#000000";

  boolean defaultBoolean = false;

  ShuffleboardLayout gridChoiceLayout = Shuffleboard.getTab("SuperShuffle")
      .getLayout("Grid Choice", BuiltInLayouts.kGrid)
      .withPosition(0, 2)
      .withSize(6, 1)
      .withProperties(Map.of("Label position", "hidden"));

  public SuperShuffle(Arm subArm) {

    this.subArm = subArm;

    createLayout();
  }

  public void createLayout() {
    ShuffleboardLayout gridLeftLayout = createGridLayout("Left Grid", gridOneColumn);
    ShuffleboardLayout gridCoopLayout = createGridLayout("Co-op Grid", gridTwoColumn);
    ShuffleboardLayout gridRightLayout = createGridLayout("Right Grid", gridThreeColumn);

    createGrid(gridLeftLayout,
        subArm::getNodeOneValue, subArm::getNodeTwoValue, subArm::getNodeThreeValue,
        subArm::getNodeFourValue, subArm::getNodeFiveValue, subArm::getNodeSixValue,
        subArm::getNodeSevenValue, subArm::getNodeEightValue, subArm::getNodeNineValue);

    createGrid(gridCoopLayout,
        subArm::getNodeTenValue, subArm::getNodeElevenValue, subArm::getNodeTwelveValue,
        subArm::getNodeThirteenValue, subArm::getNodeFourteenValue, subArm::getNodeFifteenValue,
        subArm::getNodeSixteenValue, subArm::getNodeSeventeenValue, subArm::getNodeEighteenValue);

    createGrid(gridRightLayout,
        subArm::getNodeNineteenValue, subArm::getNodeTwentyValue, subArm::getNodeTwentyOneValue,
        subArm::getNodeTwentyTwoValue, subArm::getNodeTwentyThreeValue, subArm::getNodeTwentyFourValue,
        subArm::getNodeTwentyFiveValue, subArm::getNodeTwentySixValue, subArm::getNodeTwentySevenValue);

    createGridChoice("Left Grid Choice", subArm::getGridOneValue, 0);
    createGridChoice("Co-op Grid Choice", subArm::getGridTwoValue, 1);
    createGridChoice("Right Grid Choice", subArm::getGridThreeValue, 2);
  }

  public ShuffleboardLayout createGridLayout(String gridName, int gridColumn) {
    return Shuffleboard.getTab("SuperShuffle")
        .getLayout(gridName, BuiltInLayouts.kGrid)
        .withPosition(gridColumn, gridRow)
        .withSize(gridSize, gridSize)
        .withProperties(Map.of("Label position", "hidden"));
  }

  public void createGrid(
      ShuffleboardLayout gridLayout, BooleanSupplier nodeOne, BooleanSupplier nodeTwo,
      BooleanSupplier nodeThree, BooleanSupplier nodeFour, BooleanSupplier nodeFive, BooleanSupplier nodeSix,
      BooleanSupplier nodeSeven, BooleanSupplier nodeEight, BooleanSupplier nodeNine) {

    createCell(gridLayout, "Hybrid L", nodeOne, defaultBoolean, hybridColor, offColor, cellSize, 0, hybridRow);
    createCell(gridLayout, "Hybrid M", nodeTwo, defaultBoolean, hybridColor, offColor, cellSize, 1, hybridRow);
    createCell(gridLayout, "Hybrid R", nodeThree, defaultBoolean, hybridColor, offColor, cellSize, 2, hybridRow);
    createCell(gridLayout, "Cone ML", nodeFour, defaultBoolean, coneColor, offColor, cellSize, 0, midRow);
    createCell(gridLayout, "Cube MM", nodeFive, defaultBoolean, cubeColor, offColor, cellSize, 1, midRow);
    createCell(gridLayout, "Cone MR", nodeSix, defaultBoolean, coneColor, offColor, cellSize, 2, midRow);
    createCell(gridLayout, "Cone HL", nodeSeven, defaultBoolean, coneColor, offColor, cellSize, 0, highRow);
    createCell(gridLayout, "Cube HM", nodeEight, defaultBoolean, cubeColor, offColor, cellSize, 1, highRow);
    createCell(gridLayout, "Cone HR", nodeNine, defaultBoolean, coneColor, offColor, cellSize, 2, highRow);
  }

  public void createCell(
      ShuffleboardLayout layoutName, String cellName, BooleanSupplier supplier, Boolean defaultState,
      String trueColor, String falseColor, int cellSize, int cellColumn, int cellRow) {

    layoutName
        .addBoolean(cellName, supplier)
        .withWidget("Boolean Box")
        .withProperties(Map.of("colorWhenTrue", trueColor, "colorWhenFalse", falseColor))
        .withSize(cellSize, cellSize)
        .withPosition(cellColumn, cellRow);
  }

  public void createGridChoice(String gridName, BooleanSupplier supplier, int column) {
    gridChoiceLayout
        .addBoolean(gridName, supplier)
        .withWidget("Boolean Box")
        .withProperties(Map.of("colorWhenTrue", gridColor, "colorWhenFalse", offColor))
        .withSize(2, 1)
        .withPosition(column, 0);
  }

  @Override
  public void periodic() {

  }
}
