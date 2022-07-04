/**
 * Entry point for the Dafny implementation of ROS' NavFn planner.
 *
 * It loads the map and test cases from the input file and
 * runs the AStar function generated from the Dafny code.
 */

package main

import (
	"astar_navfnplanner/dafny"
	"bufio"
	"errors"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"strconv"
	"strings"
)

func Max(x, y int) int {
    if x < y {
        return y
    }
    return x
}

func printPath(path dafny.Seq) {
	it := path.Iterator()

	elem, has := it()

	if has {
		dx, dy := elem.(RealPoint).Dtor_col(), elem.(RealPoint).Dtor_row()

		fmt.Printf("[%g, %g]", dx.Float64(), dy.Float64())
		elem, has := it()

		for has {
			dx, dy := elem.(RealPoint).Dtor_col(), elem.(RealPoint).Dtor_row()

			fmt.Printf(", [%g, %g]", dx.Float64(), dy.Float64())
			elem, has = it()
		}
	}
}

func printNavfn(navfn dafny.Seq) {
	for i := 0; i < navfn.LenInt(); i++ {
		row := navfn.IndexInt(i).(dafny.Seq)
		for j := 0;  j < row.LenInt(); j++ {
			value_raw := row.IndexInt(j).(RealInf);
			value := 1e10
			if value_raw.Is_Real() {
				value = value_raw.Dtor_r().Float64()
			}
			if i == 0 && j == 0 {
				fmt.Printf("%g", value)
			} else {
				fmt.Printf(", %g", value)
			}
		}
	}
}

func readFile(reader io.Reader, base string, printNavFn bool) (error) {
	scanner := bufio.NewReader(reader)

	// Width
	width_s, err := scanner.ReadString(' ')

	if err != nil {
		return errors.New("width expected, nothing found")
	}

	width, err := strconv.Atoi(strings.TrimSpace(width_s))

	if err != nil {
		return err
	}

	// Height
	height_s, err := scanner.ReadString('\n')

	if err != nil {
		return errors.New("height expected, nothing found")
	}

	height, err := strconv.Atoi(strings.TrimSpace(height_s))

	if err != nil {
		return err
	}

	// Filename
	filename, err := scanner.ReadString('\n')

	if err != nil {
		return errors.New("file name expected, nothing found")
	}

	costmap, err := os.ReadFile(filepath.Join(base, strings.TrimSpace(filename)))

	if err != nil {
		return err
	}

	// Test cases
	line, err := scanner.ReadString('\n')

	for err == nil {
		positions_s := strings.Split(strings.TrimSpace(line), " ")

		if len(positions_s) > 1 && positions_s[0] != "-1" {
			var positions [4]int

			for i := 0; i < 4; i++ {
				positions[i], err = strconv.Atoi(positions_s[i])

				if err != nil {
					return err
				}
			}

			start := Companion_Pose_.Create_Pose_(Companion_Point_.Create_Point_(dafny.IntOf(positions[1]), dafny.IntOf(positions[0])))
			goal := Companion_Pose_.Create_Pose_(Companion_Point_.Create_Point_(dafny.IntOf(positions[3]), dafny.IntOf(positions[2])))
			costMap := Companion_CostMap_.Create_CostMap_(func (p Point) dafny.Real {
				index := p.Dtor_col().Int() + p.Dtor_row().Int() * width
				return dafny.RealOf(float64(int(costmap[index])))
			}, dafny.IntOf(height), dafny.IntOf(width))
			numIterations := dafny.IntOf(Max(width * height / 20, width + height))
			numPathIterations := dafny.IntOf(4 * width)

			// The complete AStar algorithm
			err, path, navfn := Companion_Default___.AStar(start, goal, costMap, numIterations, numPathIterations)

			// Print a JSON object
			fmt.Printf("{\"initial\": [%d, %d], \"goal\": [%d, %d], ", positions[0], positions[1], positions[2], positions[3])

			if err {
				path = dafny.EmptySeq
			}

			fmt.Print("\"path\": [")
			printPath(path)
			if printNavFn {
				fmt.Print("], \"navfn\": [")
				printNavfn(navfn)
			}
			fmt.Println("]}")
		}

		line, err = scanner.ReadString('\n')
	}

	return nil
}

func main() {
	filename := os.Args[1]
	printNavFn := true

	if filename == "--no-navfn" {
		filename = os.Args[2]
		printNavFn = false
	}

	file, err := os.Open(filename)

	if err != nil {
		panic(err)
	}

	err = readFile(file, filepath.Dir(filename), printNavFn)

	if err != nil {
		panic(err)
	}
}
