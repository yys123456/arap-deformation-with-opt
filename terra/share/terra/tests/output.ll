; ModuleID = 'terra'
source_filename = "terra"
target datalayout = "e-m:w-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-pc-windows-msvc"

@str = private unnamed_addr constant [12 x i8] c"hello world\00"

; Function Attrs: nounwind
define void @main() local_unnamed_addr #0 {
entry:
  %puts = tail call i32 @puts(i8* getelementptr inbounds ([12 x i8], [12 x i8]* @str, i64 0, i64 0))
  ret void
}

; Function Attrs: nounwind
declare i32 @puts(i8* nocapture readonly) local_unnamed_addr #0

attributes #0 = { nounwind }
