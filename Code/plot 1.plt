set terminal pdf 
set output "C:\\Users\\Pankaj\\Desktop\\path1.pdf"
set xlabel "Column"
set ylabel "Row"
set grid
set xrange [0:7]
set yrange [0:7]
set xtics (0,1,2,3,4,5,6,7)
set ytics (0,1,2,3,4,5,6,7)
set mytics 10
plot "C:\\Users\\Pankaj\\Desktop\\gnu.txt" using ($2+0.5):($1+0.5) title 'Magnet Position' with points pt 9
#For reuse change the path or give relative path.#
