// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperShuffle extends SubsystemBase {

  Arm subArm;

  ShuffleboardTab test = Shuffleboard.getTab("SuperShuffle");

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
  String offColor = "#000000";

  boolean defaultBoolean = false;

  ShuffleboardLayout gridLeftLayout = Shuffleboard.getTab("SuperShuffle")
      .getLayout("Left Grid", BuiltInLayouts.kGrid)
      .withPosition(gridOneColumn, gridRow)
      .withSize(gridSize, gridSize)
      .withProperties(Map.of("Label position", "HIDDEN"));

  ShuffleboardLayout gridCoopLayout = Shuffleboard.getTab("SuperShuffle")
      .getLayout("Co-op Grid", BuiltInLayouts.kGrid)
      .withPosition(gridTwoColumn, gridRow)
      .withSize(gridSize, gridSize)
      .withProperties(Map.of("Label position", "HIDDEN"));

  ShuffleboardLayout gridRightLayout = Shuffleboard.getTab("SuperShuffle")
      .getLayout("Right Grid", BuiltInLayouts.kGrid)
      .withPosition(gridThreeColumn, gridRow)
      .withSize(gridSize, gridSize)
      .withProperties(Map.of("Label position", "HIDDEN"));

  public SuperShuffle(Arm subArm) {

    this.subArm = subArm;

    createGridLayout();
  }

  public void createGridLayout() {

    createGrid(gridLeftLayout);
    createGrid(gridCoopLayout);
    createGrid(gridRightLayout);
  }

  public void createGrid(ShuffleboardLayout gridLayout) {

    createCell(gridLayout, "Hybrid L", subArm::getNodeOneValue, defaultBoolean, hybridColor, offColor, cellSize, 0,
        hybridRow);
    createCell(gridLayout, "Hybrid M", subArm::getNodeTwoValue, defaultBoolean, hybridColor, offColor, cellSize, 1,
        hybridRow);
    createCell(gridLayout, "Hybrid R", subArm::getNodeThreeValue, defaultBoolean, hybridColor, offColor, cellSize, 2,
        hybridRow);
    createCell(gridLayout, "Cone ML", subArm::getNodeFourValue, defaultBoolean, coneColor, offColor, cellSize, 0,
        midRow);
    createCell(gridLayout, "Cube MM", subArm::getNodeFiveValue, defaultBoolean, cubeColor, offColor, cellSize, 1,
        midRow);
    createCell(gridLayout, "Cone MR", subArm::getNodeSixValue, defaultBoolean, coneColor, offColor, cellSize, 2,
        midRow);
    createCell(gridLayout, "Cone HL", subArm::getNodeSevenValue, defaultBoolean, coneColor, offColor, cellSize, 0,
        highRow);
    createCell(gridLayout, "Cube HM", subArm::getNodeEightValue, defaultBoolean, cubeColor, offColor, cellSize, 1,
        highRow);
    createCell(gridLayout, "Cone HR", subArm::getNodeNineValue, defaultBoolean, coneColor, offColor, cellSize, 2,
        highRow);
  }

  public void createCell(ShuffleboardLayout layoutName, String cellName, BooleanSupplier supplier, Boolean defaultState,
      String trueColor,
      String falseColor, int cellSize, int cellColumn, int cellRow) {

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
