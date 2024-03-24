$number=1
$a=1;
echo "before clearance"
string="hello world"
echo $string #輸出原來的值

unset $string
echo $string #輸出清除了string變數後的值


# 運算式求值 expr代替 $((運算式))

expr 3 + 2
echo $?

expr 3 % 2
echo $?

expr 3 \* 2
echo $?

exit 0

#int_str

a=1024
$a=$a+1
echo "the value of a is :$a"

b=102a
echo "the value of b is: $b"

declare -i b  #long int 
echo "the value of b is: $b"

c="" #empty value

$d=$d+1

exit 0


#操作自訂變數
#default value

echo linux $version

echo "set default-value"   

echo linux ${version:-2.6.12} #設定變數的預設值

echo the value is : $version #變數本身的值並沒有改變

$version="new version"  #將變數給予值

$echo linux ${version:-2.6.12} #再次輸出該變數的值
