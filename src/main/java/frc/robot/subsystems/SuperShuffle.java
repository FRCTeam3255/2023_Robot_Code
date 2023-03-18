// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperShuffle extends SubsystemBase {

  ShuffleboardTab test = Shuffleboard.getTab("SuperShuffle");

  int gridSize = 3;

  int gridOneColumn = 0;
  int gridTwoColumn = gridOneColumn + gridSize;
  int gridThreeColumn = gridOneColumn + gridSize * 2;

  int gridRow = 0;

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

  public SuperShuffle() {

    int cellSize = 1;

    int hybridRow = 0;
    int midRow = 1;
    int highRow = 2;

    createCell(gridLeftLayout, "Hybrid L", true, "lime", "black", cellSize, 0, hybridRow);
    createCell(gridLeftLayout, "Hybrid M", true, "lime", "black", cellSize, 1, hybridRow);
    createCell(gridLeftLayout, "Hybrid R", true, "lime", "black", cellSize, 2, hybridRow);

    createCell(gridLeftLayout, "Cone ML", true, "yellow", "black", cellSize, 0, midRow);
    createCell(gridLeftLayout, "Cube M", true, "purple", "black", cellSize, 1, midRow);
    createCell(gridLeftLayout, "Cone MR", true, "yellow", "black", cellSize, 2, midRow);

    createCell(gridLeftLayout, "Cone HL", true, "yellow", "black", cellSize, 0, highRow);
    createCell(gridLeftLayout, "Cube H", true, "purple", "black", cellSize, 1, highRow);
    createCell(gridLeftLayout, "Cone HR", true, "yellow", "black", cellSize, 2, highRow);

  }

  public void createCell(ShuffleboardLayout layoutName, String cellName, Boolean defaultState, String trueColor,
      String falseColor, int cellSize, int cellColumn, int cellRow) {

    layoutName
        .add(cellName, defaultState)
        .withWidget("Boolean Box")
        .withProperties(Map.of("colorWhenTrue", trueColor, "colorWhenFalse", falseColor))
        .withSize(cellSize, cellSize)
        .withPosition(cellColumn, cellRow);
  }

  @Override
  public void periodic() {

  }
}
