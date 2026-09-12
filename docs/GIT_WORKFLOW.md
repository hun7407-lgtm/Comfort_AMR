# Git Workflow

이 문서는 Comfort_AMR 프로젝트에서 실제로 쓰는 Git 명령만 정리한 것입니다.
Git을 처음 다룬다는 전제로 작성했습니다.

모든 명령은 프로젝트 루트에서 실행합니다.

```bash
cd /home/hun/Desktop/Comfort_AMR
```

---

## 0. 이 저장소의 설정

커밋 작성자 정보는 이 저장소에만 적용되도록 설정되어 있습니다.
다른 프로젝트에는 영향을 주지 않습니다.

```bash
git config user.name    # hun7407-lgtm
git config user.email   # nslabadmin@gmail.com
```

바꾸려면 같은 명령에 값을 덧붙입니다.

```bash
git config user.name "원하는이름"
git config user.email "원하는@메일"
```

### 인증

이 PC에는 `shju0519-cmd` 계정의 토큰이 저장되어 있습니다.
그 계정은 `hun7407-lgtm/Comfort_AMR`에 **읽기 권한만** 있으므로 push가 실패합니다.
`hun7407-lgtm` 계정으로 바꾸려면 저장된 자격증명을 지우고 다시 입력합니다.

```bash
# 저장된 자격증명 확인 (토큰이 그대로 보이므로 화면 공유 중에는 실행하지 마세요)
cat ~/.git-credentials

# 지우고 다시 입력하려면 파일에서 해당 줄을 삭제한 뒤 push를 시도하면
# Username / Password를 다시 물어봅니다.
```

Password 자리에는 계정 비밀번호가 아니라 **Personal Access Token**을 넣습니다.
GitHub의 Settings > Developer settings > Personal access tokens 에서 발급하고
`repo` 권한을 부여하세요.

---

## 1. 현재 변경사항 확인

작업을 시작하거나 끝낼 때 항상 먼저 실행합니다.

```bash
git status      # 어떤 파일이 바뀌었는지 목록으로 확인
git diff        # 아직 staging 하지 않은 변경의 실제 내용
git diff --stat # 파일별 변경 줄 수만 요약
```

이미 `git add` 한 파일의 내용을 보려면 다음을 씁니다.

```bash
git diff --staged
```

---

## 2. 변경 파일 staging

`git add`는 "이 파일을 다음 커밋에 포함하겠다"는 표시입니다.

```bash
git add configs/scenario_b2.yaml       # 파일 하나
git add comfort_core/                  # 디렉터리 전체
git add -A                             # 변경된 전체 (삭제 포함)
```

`git add -A`는 편하지만 의도치 않은 파일까지 들어갈 수 있으니
실행 전에 `git status`로 목록을 먼저 확인하세요.

staging을 취소하려면 다음과 같이 합니다. 파일 내용은 그대로 남습니다.

```bash
git restore --staged configs/scenario_b2.yaml
```

---

## 3. Commit

```bash
git commit -m "Add B2 static comfort space config"
```

커밋 메시지는 "무엇을 왜" 가 드러나게 씁니다. 예시입니다.

```
Add corridor scene generator driven by configs/common.yaml
Fix deadlock detection threshold to use displacement over a window
Import AMR USD from reference repository (see docs/AMR_DIGITAL_TWIN_REFERENCE.md)
```

방금 한 커밋의 메시지만 고치려면 다음을 씁니다.
**단, 이미 push한 커밋에는 쓰지 마세요.**

```bash
git commit --amend -m "새 메시지"
```

---

## 4. Push

```bash
git push origin dev
```

처음 만든 브랜치를 처음 올릴 때는 `-u`를 붙입니다.
이후로는 그 브랜치에서 `git push`만 쳐도 됩니다.

```bash
git push -u origin feature/amr-setup
```

---

## 5. 최신 remote 반영

```bash
git pull --rebase origin dev
```

`--rebase`를 쓰는 이유는 불필요한 merge 커밋을 만들지 않고
내 작업을 remote 최신 커밋 위에 얹기 위해서입니다. 히스토리가 깔끔해집니다.

rebase 중 충돌이 나면 Git이 멈추고 충돌 파일을 알려줍니다.
파일을 직접 고친 뒤 다음을 실행합니다.

```bash
git add <고친파일>
git rebase --continue
```

도중에 그만두고 원래 상태로 되돌리려면 다음을 씁니다. 안전합니다.

```bash
git rebase --abort
```

---

## 6. 새 feature branch 생성

```bash
git switch dev              # 먼저 dev로 이동
git pull --rebase origin dev # dev를 최신으로
git switch -c feature/b2-static
```

`switch -c`는 "새 브랜치를 만들고 그리로 이동"입니다.
현재 브랜치의 상태를 그대로 물려받습니다.

---

## 7. Branch 확인

```bash
git branch          # 로컬 브랜치 목록, 현재 브랜치에 * 표시
git branch -a       # remote 브랜치까지 포함
git branch --show-current
```

브랜치 사이 이동은 다음과 같습니다.

```bash
git switch dev
```

---

## 8. 실수로 수정한 파일 원복

### 아직 add 하지 않은 파일을 되돌리기

```bash
git restore configs/common.yaml
```

마지막 커밋 상태로 되돌립니다. **수정 내용은 사라지므로** 실행 전에
`git diff`로 무엇을 버리는지 반드시 확인하세요.

### 지금 작업을 버리지 않고 잠시 치워두기

되돌리기가 불안하면 `stash`가 더 안전합니다.
변경 내용을 임시 보관함에 넣고 작업 트리를 깨끗하게 만듭니다.

```bash
git stash push -m "b2 실험 도중 작업"
git stash list      # 보관 목록
git stash pop       # 가장 최근 것을 되돌려 꺼내기
```

### 특정 파일을 특정 커밋 시점으로 되돌리기

```bash
git log --oneline -- configs/common.yaml   # 그 파일의 커밋 이력
git restore --source <커밋해시> configs/common.yaml
```

### 이미 커밋한 것을 취소하기

히스토리를 지우지 않고 "되돌리는 커밋"을 새로 만드는 방식이 안전합니다.

```bash
git revert <커밋해시>
```

---

## 쓰지 않는 명령

아래 두 명령은 이 프로젝트의 기본 workflow에서 사용하지 않습니다.

| 명령 | 왜 위험한가 |
|---|---|
| `git reset --hard` | 커밋되지 않은 작업을 복구 불가능하게 삭제합니다. |
| `git push --force` | remote 히스토리를 덮어써서 다른 사람의 작업을 지웁니다. |

되돌려야 하는 상황에서는 위 8번의 `restore`, `stash`, `revert`를 쓰세요.
정말 `reset --hard`가 필요한 경우가 생기면 먼저 `git stash`로 백업한 뒤에 하세요.

---

## Branch 전략

```
main                    안정 버전. 검증된 결과만 올라갑니다.
 └── dev                통합 브랜치. 평소 작업은 여기서 시작합니다.
      ├── feature/amr-setup
      ├── feature/b2-static
      └── feature/b3-adaptive
```

feature 브랜치 작업이 끝나면 `dev`로 병합합니다.

```bash
git switch dev
git merge --no-ff feature/b2-static
```

`--no-ff`는 병합 기록을 커밋으로 남겨서
"이 기능이 언제 들어왔는지"를 나중에 추적할 수 있게 합니다.

---

## 자동 push를 하지 않는 이유

이 프로젝트에서 AI 도구는 절대 자동으로 push하지 않습니다.
코드 변경 후에는 항상 아래를 직접 확인한 뒤 본인이 push합니다.

```bash
git status
git diff
```
