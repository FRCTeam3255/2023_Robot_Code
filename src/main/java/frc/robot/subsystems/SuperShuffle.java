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

  int gridWidth = 2;
  int gridHeight = 3;
  int gridRow = 0;

  int hybridRow = 0;
  int midRow = 1;
  int highRow = 2;

  int gridOneColumn = 5;

  int cellSize = 1;
  int cellColumnOneOffset = 1;
  int cellColumnTwoOffset = 2;

  String coneColor = "#fffb00";
  String cubeColor = "#a300c4";
  String hybridColor = "#00ff00";
  String gridColor = "#ffffff";
  String offColor = "#000000";

  boolean defaultBoolean = false;

  public SuperShuffle(Arm subArm) {

    this.subArm = subArm;

    createLayout();
  }

  public void createLayout() {
    ShuffleboardLayout gridLeftLayout = createGridLayout("Node Selection", gridOneColumn);

    createGrid(gridLeftLayout,
        subArm::getNodeSixValue, subArm::getNodeFiveValue, subArm::getNodeFourValue, subArm::getNodeThreeValue,
        subArm::getNodeTwelveValue, subArm::getNodeOneValue);
  }

  public ShuffleboardLayout createGridLayout(String gridName, int gridColumn) {
    return Shuffleboard.getTab("SuperShuffle")
        .getLayout(gridName, BuiltInLayouts.kGrid)
        .withPosition(gridColumn, gridRow)
        .withSize(gridWidth, gridHeight)
        .withProperties(Map.of("Label position", "hidden"));
  }

  public void createGrid(
      ShuffleboardLayout gridLayout, BooleanSupplier nodeOne, BooleanSupplier nodeTwo, BooleanSupplier nodeThree,
      BooleanSupplier nodeFour, BooleanSupplier nodeFive, BooleanSupplier nodeSix) {

    createCell(gridLayout, "Hybrid L", nodeOne, defaultBoolean, hybridColor, offColor, cellSize, cellColumnOneOffset,
        hybridRow);
    createCell(gridLayout, "Hybrid M", nodeTwo, defaultBoolean, hybridColor, offColor, cellSize, cellColumnTwoOffset,
        hybridRow);
    createCell(gridLayout, "Hybrid R", nodeThree, defaultBoolean, hybridColor, offColor, cellSize, cellColumnOneOffset,
        midRow);
    createCell(gridLayout, "Cone ML", nodeFour, defaultBoolean, coneColor, offColor, cellSize, cellColumnTwoOffset,
        midRow);
    createCell(gridLayout, "Cube MM", nodeFive, defaultBoolean, cubeColor, offColor, cellSize, cellColumnOneOffset,
        highRow);
    createCell(gridLayout, "Cone MR", nodeSix, defaultBoolean, coneColor, offColor, cellSize, cellColumnTwoOffset,
        highRow);
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

  @Override
  public void periodic() {

  }
}
